# Phase 1 Setup Guide - External Services

This guide walks you through setting up external services for the RAG chatbot.

## Prerequisites

- Completed Phase 0 (backend structure created)
- Python virtual environment activated

## Step 1: Get Google Gemini API Key (T015)

### Instructions

1. Visit: https://aistudio.google.com/app/apikey
2. Sign in with your Google account
3. Click "Create API Key"
4. Copy the API key

### Add to .env

Create `backend/.env` file (copy from `.env.example`):

```bash
cd backend
cp .env.example .env
```

Edit `.env` and add:

```
GEMINI_API_KEY=your-actual-api-key-here
GEMINI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/
GEMINI_MODEL=gemini-2.5-flash
```

---

## Step 2: Get Cohere API Key (T015)

### Instructions

1. Visit: https://dashboard.cohere.com/api-keys
2. Sign up for free account
3. Navigate to API Keys section
4. Copy your API key (or create new one)

### Add to .env

Edit `backend/.env` and add:

```
COHERE_API_KEY=your-actual-cohere-key-here
COHERE_EMBEDDING_MODEL=embed-english-v3.0
```

---

## Step 3: Setup Qdrant Cloud (T009)

### Instructions

1. Visit: https://cloud.qdrant.io/
2. Sign up for free account (no credit card required)
3. Create a cluster:
   - Click "Create Cluster"
   - Select "Free Tier" (1GB storage)
   - Choose region closest to you
   - Name: `humanoid-robotics`
4. Once cluster is created:
   - Click on cluster name
   - Copy "Cluster URL" (looks like: `https://xxx.qdrant.io`)
   - Go to "API Keys" tab
   - Create API key, copy it

### Add to .env

Edit `backend/.env` and add:

```
QDRANT_URL=https://your-cluster-id.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION_NAME=humanoid_robotics_book
```

### Run Setup Script (T010)

```bash
# Install qdrant-client if not installed
pip install qdrant-client

# Run setup script
python scripts/setup_qdrant.py
```

Expected output:
```
🔌 Connecting to Qdrant Cloud...
📦 Creating collection 'humanoid_robotics_book'...
✅ Collection created successfully!
✨ Qdrant setup complete!
```

### Test Connection (T011)

```bash
python scripts/test_qdrant.py
```

Expected output:
```
🔌 Connecting to Qdrant Cloud...
📦 Checking collection 'humanoid_robotics_book'...
✅ Collection found: 0 points
🧪 Inserting test vector...
✅ Test vector inserted
🔍 Searching for test vector...
✅ Test vector found!
🗑️  Cleaning up test data...
✅ Test data deleted
✨ All Qdrant tests passed!
```

---

## Step 4: Setup Neon Postgres (T012)

### Instructions

1. Visit: https://neon.tech/
2. Sign up for free account
3. Create a project:
   - Click "Create Project"
   - Name: `rag-chatbot`
   - Region: Choose closest to you
   - Postgres version: 15 (default)
4. Once project is created:
   - Go to "Dashboard"
   - Find "Connection string"
   - Copy the connection string (looks like: `postgresql://user:password@hostname/database`)

### Add to .env

Edit `backend/.env` and add:

```
NEON_DATABASE_URL=postgresql://your-actual-connection-string-here
```

### Run Setup Script (T013)

```bash
# Install asyncpg if not installed
pip install asyncpg

# Run setup script
python scripts/setup_postgres.py
```

Expected output:
```
🔌 Connecting to Neon Postgres...
📦 Creating 'users' table...
✅ 'users' table created
📦 Creating 'conversations' table...
✅ 'conversations' table created
📦 Creating 'user_progress' table...
✅ 'user_progress' table created
📊 Creating indexes...
✅ Indexes created
✨ Postgres schema setup complete!
```

### Test Connection (T014)

```bash
python scripts/test_postgres.py
```

Expected output:
```
🔌 Connecting to Neon Postgres...
🧪 Testing 'users' table...
✅ Test user created
✅ Test user retrieved
🧪 Testing 'conversations' table...
✅ Test conversation created
🧪 Testing 'user_progress' table...
✅ Test progress record created
🗑️  Cleaning up test data...
✅ Test data deleted
✨ All Postgres tests passed!
```

---

## Step 5: Add CORS Origins

Edit `backend/.env` and add:

```
CORS_ORIGINS=https://anusbutt.github.io,http://localhost:3000
API_RATE_LIMIT=100
```

---

## Verification Checklist

After completing all steps, your `.env` file should have:

- [x] GEMINI_API_KEY
- [x] GEMINI_BASE_URL
- [x] GEMINI_MODEL
- [x] COHERE_API_KEY
- [x] COHERE_EMBEDDING_MODEL
- [x] QDRANT_URL
- [x] QDRANT_API_KEY
- [x] QDRANT_COLLECTION_NAME
- [x] NEON_DATABASE_URL
- [x] CORS_ORIGINS
- [x] API_RATE_LIMIT

All test scripts should pass:
- [x] `python scripts/test_qdrant.py` → All tests passed
- [x] `python scripts/test_postgres.py` → All tests passed

---

## Troubleshooting

### Qdrant Connection Fails

- Check Qdrant cluster status in dashboard (should be "Running")
- Verify API key is correct (regenerate if needed)
- Check firewall/network (Qdrant uses HTTPS port 443)

### Postgres Connection Fails

- Check connection string format: `postgresql://user:password@host/db`
- Verify Neon project is active (not suspended)
- Check if IP is allowed (Neon allows all IPs by default)

### API Keys Not Working

- Regenerate API keys in respective dashboards
- Check for trailing spaces in `.env` file
- Ensure `.env` file is in `backend/` directory (not project root)

---

## Next Steps

Once all external services are configured and tests pass, proceed to:

**Phase 1 Implementation Tasks (T016-T020)**:
- Implement Qdrant service wrapper
- Implement Cohere service wrapper
- Implement Postgres service wrapper
- Create Pydantic models
- Create domain entities

Then you're ready for **Phase 2: Content Embedding Pipeline**!
