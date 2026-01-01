"""
RAG Service - Retrieval-Augmented Generation

Orchestrates the RAG pipeline: query embedding, vector search,
context building, and LLM generation with Gemini.
"""

from typing import List, Dict, Any, Optional
import logging
from openai import AsyncOpenAI
from app.services.cohere_service import CohereService
from app.services.qdrant_service import QdrantService
from app.models.schemas import Message, Source

logger = logging.getLogger(__name__)


class RAGService:
    """
    Service for RAG (Retrieval-Augmented Generation) chatbot.

    Handles query embedding, context retrieval, prompt building,
    and LLM generation using Gemini via OpenAI-compatible API.
    """

    def __init__(
        self,
        cohere_service: CohereService,
        qdrant_service: QdrantService,
        gemini_api_key: str,
        gemini_base_url: str,
        gemini_model: str = "gemini-2.0-flash-exp"
    ):
        """
        Initialize RAG service.

        Args:
            cohere_service: Service for generating embeddings
            qdrant_service: Service for vector search
            gemini_api_key: Gemini API key
            gemini_base_url: Gemini API base URL
            gemini_model: Gemini model name
        """
        self.cohere = cohere_service
        self.qdrant = qdrant_service
        self.gemini_model = gemini_model

        # Initialize Gemini client via OpenAI-compatible API
        self.gemini_client = AsyncOpenAI(
            api_key=gemini_api_key,
            base_url=gemini_base_url
        )

        logger.info(f"RAG service initialized with model: {gemini_model}")

    async def query(
        self,
        user_query: str,
        conversation_history: List[Message] = None,
        selected_text: Optional[str] = None,
        lesson_id: Optional[str] = None,
        top_k: int = 5,
        score_threshold: float = 0.7
    ) -> Dict[str, Any]:
        """
        Process a user query through the RAG pipeline.

        Args:
            user_query: User's question
            conversation_history: Last 5 messages for context
            selected_text: Text selected by user (optional)
            lesson_id: Current lesson ID for filtering (optional)
            top_k: Number of chunks to retrieve
            score_threshold: Minimum similarity score

        Returns:
            {
                'response': str,
                'sources': List[Source],
                'retrieved_chunks': int,
                'used_selected_text': bool
            }
        """
        logger.info(f"Processing query: {user_query[:100]}...")

        # Validate and sanitize selected text
        sanitized_selected_text = None
        if selected_text:
            sanitized_selected_text = self._validate_selected_text(selected_text)
            if sanitized_selected_text:
                logger.info(f"Using selected text ({len(sanitized_selected_text)} chars)")

        # Step 1: Build search query (combine user query + selected text if provided)
        search_query = user_query
        if sanitized_selected_text:
            # Enhance search query with selected text context
            search_query = f"{user_query} Context: {sanitized_selected_text[:200]}"

        # Step 2: Embed the query
        try:
            query_embedding = self.cohere.embed_query(search_query)
            logger.info(f"Query embedded (dim: {len(query_embedding)})")
        except Exception as e:
            logger.error(f"Embedding failed: {e}")
            # Fallback: use mock embedding for testing
            logger.warning("Using mock embedding due to API failure")
            import random
            query_embedding = [random.uniform(-1, 1) for _ in range(1024)]

        # Step 3: Build filters if lesson_id provided
        filters = None
        if lesson_id:
            filters = {'lesson': lesson_id}

        # Step 4: Adjust retrieval parameters if selected text is provided
        # Retrieve more chunks for selected text queries to ensure coverage
        adjusted_top_k = top_k + 3 if sanitized_selected_text else top_k
        adjusted_threshold = score_threshold - 0.1 if sanitized_selected_text else score_threshold

        # Step 5: Search Qdrant for relevant chunks
        results = self.qdrant.search(
            query_vector=query_embedding,
            limit=adjusted_top_k,
            score_threshold=max(0.3, adjusted_threshold),  # Don't go below 0.3
            filters=filters
        )

        logger.info(f"Retrieved {len(results)} chunks from Qdrant")

        # Step 6: Build context from retrieved chunks
        context = self._build_context(results, sanitized_selected_text)

        # Step 7: Build conversation context
        conversation_context = self._build_conversation_context(conversation_history)

        # Step 8: Generate response with Gemini
        response_text = await self._generate_response(
            user_query=user_query,
            context=context,
            conversation_context=conversation_context,
            has_selected_text=sanitized_selected_text is not None
        )

        # Step 9: Extract sources
        sources = self._extract_sources(results)

        return {
            'response': response_text,
            'sources': sources,
            'retrieved_chunks': len(results),
            'used_selected_text': sanitized_selected_text is not None
        }

    def _validate_selected_text(self, selected_text: str) -> Optional[str]:
        """
        Validate and sanitize selected text.

        Args:
            selected_text: Raw selected text from user

        Returns:
            Sanitized text or None if invalid
        """
        if not selected_text:
            return None

        # Strip whitespace
        text = selected_text.strip()

        # Check minimum length (10 characters as per spec)
        if len(text) < 10:
            logger.warning(f"Selected text too short ({len(text)} chars), ignoring")
            return None

        # Check maximum length (prevent abuse)
        MAX_SELECTED_TEXT_LENGTH = 2000
        if len(text) > MAX_SELECTED_TEXT_LENGTH:
            logger.warning(f"Selected text too long ({len(text)} chars), truncating")
            text = text[:MAX_SELECTED_TEXT_LENGTH]

        # Basic sanitization - remove potentially harmful characters
        # Allow letters, numbers, punctuation, spaces, newlines
        import re
        text = re.sub(r'[^\w\s\.,;:!?\-\'"()\[\]{}\/\n]', '', text)

        return text if len(text) >= 10 else None

    def _build_context(self, results: List[Dict], selected_text: Optional[str] = None) -> str:
        """
        Build context string from retrieved chunks.

        Args:
            results: List of search results from Qdrant
            selected_text: User-selected text (optional, already validated)

        Returns:
            Context string for LLM prompt
        """
        context_parts = []

        # Add selected text PROMINENTLY if provided
        if selected_text:
            context_parts.append(
                f"**🎯 USER SELECTED TEXT (Primary Focus):**\n"
                f"```\n{selected_text}\n```\n"
                f"**The student specifically selected the above text and wants to understand it better.**\n"
            )

        # Add retrieved chunks as supporting context
        if results:
            context_parts.append("**📚 Related Course Content:**\n")
            for i, result in enumerate(results, 1):
                # Use the formatted result structure from QdrantService
                content = result.get('content', '')
                module_title = result.get('module', '')
                lesson_title = result.get('lesson', '')
                section = result.get('section', '')

                context_parts.append(
                    f"**Source {i}: {module_title} > {lesson_title} > {section}**\n{content}\n"
                )

        return "\n---\n".join(context_parts)

    def _build_conversation_context(self, history: Optional[List[Message]]) -> str:
        """
        Build conversation context from message history.

        Uses last 5 messages maximum for context window management.
        Formats messages with clear role separation for better LLM understanding.

        Args:
            history: List of previous messages (already limited to 5 by API validator)

        Returns:
            Formatted conversation history string
        """
        if not history or len(history) == 0:
            return ""

        # Already limited to last 5 by ChatQueryRequest validator
        # But double-check as defensive programming
        recent_messages = history[-5:] if len(history) > 5 else history

        context_parts = ["**📜 Previous Conversation (Last {} turns):**\n".format(len(recent_messages))]

        for i, msg in enumerate(recent_messages, 1):
            role = "Student" if msg.role == "user" else "AI Tutor"

            # Format message with turn number for clarity
            if msg.role == "user":
                # Include selected text if present
                if msg.selected_text:
                    context_parts.append(
                        f"Turn {i} - {role}: {msg.content}\n"
                        f"  [Selected text: {msg.selected_text[:100]}...]"
                    )
                else:
                    context_parts.append(f"Turn {i} - {role}: {msg.content}")
            else:
                # For assistant messages, include truncated response
                response_preview = msg.content[:200] + "..." if len(msg.content) > 200 else msg.content
                context_parts.append(f"Turn {i} - {role}: {response_preview}")

        return "\n".join(context_parts)

    async def _generate_response(
        self,
        user_query: str,
        context: str,
        conversation_context: str,
        has_selected_text: bool = False
    ) -> str:
        """
        Generate response using Gemini LLM.

        Args:
            user_query: User's question
            context: Retrieved context from vector search
            conversation_context: Previous conversation messages
            has_selected_text: Whether user provided selected text

        Returns:
            Generated response text
        """
        # Build system prompt based on whether selected text is present
        if has_selected_text:
            system_prompt = """You are an expert AI tutor for the Physical AI & Humanoid Robotics course. Your role is to help students understand specific passages from the course materials.

**IMPORTANT - Selected Text Focus:**
The student has selected a specific passage from the course and wants to understand it better. Your response should:
1. **PRIORITIZE explaining the selected text** - this is your primary focus
2. Focus on clarifying concepts mentioned in the selected text
3. Explain technical terms used in the selected passage
4. Provide context for why this content matters in the broader course
5. Use the additional course materials as supporting references
6. Be concise but thorough - aim for 2-4 paragraphs
7. If the selected text is unclear or too short, ask for clarification

Guidelines:
- Answer based on the selected text AND provided course materials
- Explain concepts at an appropriate level for a student learning this topic
- Reference specific parts of the selected text in your explanation
- Use analogies when helpful for complex concepts"""
        else:
            system_prompt = """You are an expert AI tutor for the Physical AI & Humanoid Robotics course. Your role is to help students learn ROS2, sensors, perception, and humanoid robot programming.

Guidelines:
1. Answer based ONLY on the provided context from the course materials
2. If the context doesn't contain enough information, say "I don't have enough information in the course materials to answer that fully"
3. Be concise but thorough - aim for 2-4 paragraphs
4. Use technical terms correctly and explain them when first mentioned
5. Reference specific modules/lessons when relevant
6. If the user asks about something off-topic, politely redirect to course topics
7. For code-related questions, provide practical examples when possible"""

        # Build user prompt with context
        if has_selected_text:
            user_prompt = f"""Context from course materials:

{context}

{conversation_context if conversation_context else ""}

Student Question about the selected text: {user_query}

Please explain the selected text above and answer the student's question. Focus primarily on explaining the selected passage."""
        else:
            user_prompt = f"""Context from course materials:

{context}

{conversation_context if conversation_context else ""}

Student Question: {user_query}

Please provide a helpful answer based on the context above."""

        try:
            # Call Gemini via OpenAI-compatible API
            response = await self.gemini_client.chat.completions.create(
                model=self.gemini_model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                temperature=0.7,
                max_tokens=800
            )

            response_text = response.choices[0].message.content
            logger.info(f"Generated response ({len(response_text)} chars)")

            return response_text

        except Exception as e:
            logger.error(f"Gemini API error: {e}")
            raise

    def _extract_sources(self, results: List[Dict]) -> List[Source]:
        """
        Extract source citations from search results.

        Args:
            results: List of search results from Qdrant

        Returns:
            List of Source objects
        """
        sources = []
        seen = set()  # Track unique module+lesson combinations

        for result in results:
            # Use the formatted result structure from QdrantService
            module_title = result.get('module', '')
            lesson_title = result.get('lesson', '')
            section = result.get('section', '')

            # Create unique key for deduplication
            key = f"{module_title}|{lesson_title}"

            if key not in seen:
                sources.append(Source(
                    module=module_title,
                    lesson=lesson_title,
                    section=section if section else None
                ))
                seen.add(key)

        return sources
