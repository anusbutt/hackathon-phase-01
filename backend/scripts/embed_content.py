"""
Content Embedding Pipeline

Extracts content from MDX/MD files, chunks it, generates embeddings,
and uploads to Qdrant vector database.

Usage:
    python scripts/embed_content.py --module all
    python scripts/embed_content.py --module 01-ros2-nervous-system
"""

import sys
import os
import re
import yaml
import argparse
import time
from pathlib import Path
from typing import List, Dict, Any, Optional
import uuid

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from dotenv import load_dotenv
from app.services.cohere_service import CohereService
from app.services.qdrant_service import QdrantService

load_dotenv()

# Constants
BOOK_SOURCE_PATH = Path(__file__).parent.parent.parent / "book-source" / "docs" / "13-Physical-AI-Humanoid-Robotics"
CHUNK_SIZE_MIN = 500  # Minimum tokens per chunk
CHUNK_SIZE_MAX = 1000  # Maximum tokens per chunk


def estimate_tokens(text: str) -> int:
    """Estimate token count (rough approximation: 1 token ≈ 4 characters)."""
    return len(text) // 4


def parse_frontmatter(content: str) -> tuple[Dict[str, Any], str]:
    """
    Parse YAML frontmatter from markdown file.

    Returns:
        (metadata_dict, markdown_content)
    """
    # Match frontmatter between --- markers
    frontmatter_pattern = r'^---\s*\n(.*?)\n---\s*\n(.*)$'
    match = re.match(frontmatter_pattern, content, re.DOTALL)

    if not match:
        return {}, content

    frontmatter_text = match.group(1)
    markdown_content = match.group(2)

    try:
        metadata = yaml.safe_load(frontmatter_text)
        return metadata or {}, markdown_content
    except yaml.YAMLError as e:
        print(f"[WARNING] YAML parse error: {e}")
        return {}, markdown_content


def extract_sections(markdown: str) -> List[Dict[str, str]]:
    """
    Split markdown into sections by ## headings.

    Returns:
        List of {heading: str, content: str}
    """
    sections = []

    # Split by ## headings (not # main title)
    section_pattern = r'^## (.+?)$'
    parts = re.split(section_pattern, markdown, flags=re.MULTILINE)

    # First part is intro (before first ##)
    if parts[0].strip():
        sections.append({
            'heading': 'Introduction',
            'content': parts[0].strip()
        })

    # Remaining parts alternate: heading, content, heading, content...
    for i in range(1, len(parts), 2):
        if i + 1 < len(parts):
            sections.append({
                'heading': parts[i].strip(),
                'content': parts[i + 1].strip()
            })

    return sections


def chunk_section(section: Dict[str, str]) -> List[str]:
    """
    Chunk a section into 500-1000 token pieces.

    Strategy:
    - If section < 500 tokens: return as single chunk
    - If 500-1000 tokens: return as single chunk
    - If > 1000 tokens: split by paragraphs, aiming for 500-1000 per chunk
    """
    content = section['content']
    heading = section['heading']
    tokens = estimate_tokens(content)

    # Small section - keep as one chunk
    if tokens <= CHUNK_SIZE_MAX:
        return [f"## {heading}\n\n{content}"]

    # Large section - split by paragraphs
    paragraphs = content.split('\n\n')
    chunks = []
    current_chunk = f"## {heading}\n\n"
    current_tokens = estimate_tokens(current_chunk)

    for paragraph in paragraphs:
        paragraph = paragraph.strip()
        if not paragraph:
            continue

        para_tokens = estimate_tokens(paragraph)

        # If adding this paragraph would exceed max, start new chunk
        if current_tokens + para_tokens > CHUNK_SIZE_MAX and current_tokens >= CHUNK_SIZE_MIN:
            chunks.append(current_chunk.strip())
            current_chunk = f"## {heading} (continued)\n\n{paragraph}\n\n"
            current_tokens = estimate_tokens(current_chunk)
        else:
            current_chunk += f"{paragraph}\n\n"
            current_tokens += para_tokens

    # Add remaining chunk
    if current_chunk.strip():
        chunks.append(current_chunk.strip())

    return chunks


def extract_lesson_metadata(frontmatter: Dict[str, Any], module_id: str, lesson_id: str) -> Dict[str, Any]:
    """
    Extract relevant metadata for Qdrant storage.

    Returns metadata dict with keys needed for ContentChunk.
    """
    # Extract bloom levels from skills
    bloom_levels = []
    if 'skills' in frontmatter and isinstance(frontmatter['skills'], list):
        for skill in frontmatter['skills']:
            if isinstance(skill, dict) and 'bloom_level' in skill:
                bloom_levels.append(skill['bloom_level'])

    # Use first bloom level or default to 'understand'
    bloom_level = bloom_levels[0] if bloom_levels else 'understand'

    # Extract tags
    tags = frontmatter.get('tags', [])
    if isinstance(tags, str):
        tags = [tags]

    # Extract skills names
    skills = []
    if 'skills' in frontmatter and isinstance(frontmatter['skills'], list):
        for skill in frontmatter['skills']:
            if isinstance(skill, dict) and 'name' in skill:
                skills.append(skill['name'])

    return {
        'module': module_id,
        'lesson': lesson_id,
        'lesson_title': frontmatter.get('title', ''),
        'module_title': get_module_title(module_id),
        'tags': tags,
        'skills': skills,
        'bloom_level': bloom_level
    }


def get_module_title(module_id: str) -> str:
    """Map module ID to human-readable title."""
    module_titles = {
        '01-ros2-nervous-system': 'Module 1: ROS2 Nervous System',
        '02-sensors-perception': 'Module 2: Sensors & Perception',
        '03-isaac-ai-brain': 'Module 3: Isaac AI Brain',
        '04-vision-language-action': 'Module 4: Vision-Language-Action Models'
    }
    return module_titles.get(module_id, module_id)


def process_lesson(
    lesson_path: Path,
    module_id: str,
    cohere_service: CohereService,
    qdrant_service: QdrantService,
    dry_run: bool = False
) -> int:
    """
    Process a single lesson file: extract, chunk, embed, upload.

    Returns:
        Number of chunks created
    """
    lesson_id = lesson_path.stem  # e.g., "01-ros2-fundamentals"

    print(f"\n[INFO] Processing {module_id}/{lesson_id}...")

    # Read file
    with open(lesson_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Parse frontmatter
    frontmatter, markdown = parse_frontmatter(content)

    # Extract metadata
    lesson_metadata = extract_lesson_metadata(frontmatter, module_id, lesson_id)

    # Extract sections
    sections = extract_sections(markdown)
    print(f"[INFO] Found {len(sections)} sections")

    # Chunk sections
    all_chunks = []
    for section in sections:
        section_chunks = chunk_section(section)
        for chunk_text in section_chunks:
            all_chunks.append({
                'text': chunk_text,
                'section': section['heading'],
                'metadata': lesson_metadata
            })

    print(f"[INFO] Created {len(all_chunks)} chunks")

    if dry_run:
        print(f"[DRY RUN] Would upload {len(all_chunks)} chunks")
        for i, chunk in enumerate(all_chunks[:3]):  # Show first 3
            print(f"\n  Chunk {i+1} preview (tokens: {estimate_tokens(chunk['text'])}):")
            print(f"  Section: {chunk['section']}")
            print(f"  Text: {chunk['text'][:150]}...")
        return len(all_chunks)

    # Generate embeddings with rate limiting
    print(f"[INFO] Generating embeddings (batched with rate limiting)...")
    chunk_texts = [chunk['text'] for chunk in all_chunks]

    # Process ONE chunk at a time to avoid rate limits (Cohere free tier is VERY strict)
    BATCH_SIZE = 1  # Single chunk at a time
    embeddings = []

    for i in range(0, len(chunk_texts), BATCH_SIZE):
        batch = chunk_texts[i:i + BATCH_SIZE]
        batch_num = (i // BATCH_SIZE) + 1
        total_batches = (len(chunk_texts) + BATCH_SIZE - 1) // BATCH_SIZE

        print(f"[INFO] Processing chunk {batch_num}/{total_batches}...")

        # Retry logic for rate limits
        max_retries = 5
        retry_delay = 10  # Start with 10 seconds

        for attempt in range(max_retries):
            try:
                batch_embeddings = cohere_service.embed_documents(batch)
                embeddings.extend(batch_embeddings)
                print(f"[SUCCESS] Chunk {batch_num} embedded")
                break
            except Exception as e:
                if "429" in str(e) or "TooManyRequestsError" in str(type(e).__name__):
                    if attempt < max_retries - 1:
                        wait_time = retry_delay * (2 ** attempt)  # Exponential backoff
                        print(f"[WARNING] Rate limit hit. Waiting {wait_time}s before retry {attempt+1}/{max_retries}...")
                        time.sleep(wait_time)
                    else:
                        print(f"[ERROR] Rate limit persists after {max_retries} retries")
                        raise
                else:
                    print(f"[ERROR] Embedding failed: {e}")
                    raise

        # Add delay between chunks to respect rate limits (60 seconds = ~1 per minute)
        if i + BATCH_SIZE < len(chunk_texts):
            time.sleep(60)  # 60 seconds between chunks for free tier

    # Upload to Qdrant
    print(f"[INFO] Uploading to Qdrant...")
    points = []
    for i, (chunk, embedding) in enumerate(zip(all_chunks, embeddings)):
        chunk_id = str(uuid.uuid4())

        payload = {
            'chunk_id': chunk_id,
            'content': chunk['text'],
            'section': chunk['section'],
            **chunk['metadata']
        }

        points.append({
            'id': chunk_id,
            'vector': embedding,
            'payload': payload
        })

    qdrant_service.client.upsert(
        collection_name=qdrant_service.collection_name,
        points=points
    )

    print(f"[SUCCESS] Uploaded {len(points)} chunks for {lesson_id}")
    return len(all_chunks)


def process_module(
    module_id: str,
    cohere_service: CohereService,
    qdrant_service: QdrantService,
    dry_run: bool = False
) -> int:
    """
    Process all lessons in a module.

    Returns:
        Total number of chunks created
    """
    module_path = BOOK_SOURCE_PATH / module_id

    if not module_path.exists():
        print(f"[ERROR] Module not found: {module_path}")
        return 0

    print(f"\n{'='*60}")
    print(f"Processing {get_module_title(module_id)}")
    print(f"{'='*60}")

    # Find all lesson files (exclude .summary.md, quiz, README)
    lesson_files = []
    for file in sorted(module_path.glob('*.md')):
        if '.summary' in file.name or file.name in ['quiz.md', 'README.md']:
            continue
        lesson_files.append(file)

    print(f"[INFO] Found {len(lesson_files)} lessons")

    total_chunks = 0
    for lesson_file in lesson_files:
        chunks = process_lesson(
            lesson_file,
            module_id,
            cohere_service,
            qdrant_service,
            dry_run
        )
        total_chunks += chunks

    return total_chunks


def main():
    parser = argparse.ArgumentParser(description='Embed book content into Qdrant')
    parser.add_argument(
        '--module',
        type=str,
        default='all',
        help='Module to process (e.g., "01-ros2-nervous-system" or "all")'
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Run without uploading to Qdrant (test extraction only)'
    )
    args = parser.parse_args()

    # Initialize services
    cohere_api_key = os.getenv('COHERE_API_KEY')
    cohere_model = os.getenv('COHERE_EMBEDDING_MODEL', 'embed-english-v3.0')
    qdrant_url = os.getenv('QDRANT_URL')
    qdrant_api_key = os.getenv('QDRANT_API_KEY')
    qdrant_collection = os.getenv('QDRANT_COLLECTION_NAME', 'humanoid_robotics_book')

    if not all([cohere_api_key, qdrant_url, qdrant_api_key]):
        print("[ERROR] Missing environment variables!")
        print("Required: COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY")
        sys.exit(1)

    print("[INFO] Initializing services...")
    cohere_service = CohereService(cohere_api_key, cohere_model)
    qdrant_service = QdrantService(qdrant_url, qdrant_api_key, qdrant_collection)

    # Process modules
    modules = [
        '01-ros2-nervous-system',
        '02-sensors-perception',
        '03-isaac-ai-brain',
        '04-vision-language-action'
    ]

    if args.module != 'all':
        if args.module not in modules:
            print(f"[ERROR] Invalid module: {args.module}")
            print(f"Valid options: {', '.join(modules + ['all'])}")
            sys.exit(1)
        modules = [args.module]

    # Process
    total_chunks = 0
    for module_id in modules:
        chunks = process_module(module_id, cohere_service, qdrant_service, args.dry_run)
        total_chunks += chunks

    # Summary
    print(f"\n{'='*60}")
    print(f"SUMMARY")
    print(f"{'='*60}")
    print(f"Total chunks: {total_chunks}")
    if args.dry_run:
        print(f"[DRY RUN] No data uploaded")
    else:
        print(f"[SUCCESS] All chunks uploaded to Qdrant!")
        print(f"Collection: {qdrant_collection}")


if __name__ == '__main__':
    main()
