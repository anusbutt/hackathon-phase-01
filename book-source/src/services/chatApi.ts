/**
 * API client for RAG chatbot backend
 * Handles all HTTP requests to the FastAPI backend
 */

import type {
  ChatQueryRequest,
  ChatQueryResponse,
  ChatError,
  HealthCheckResponse,
} from '../types/chat';

/**
 * Get backend API URL from environment or default to Railway URL
 */
const getBackendUrl = (): string => {
  // For local development, use localhost
  if (typeof window !== 'undefined' && window.location.hostname === 'localhost') {
    return 'http://localhost:8001';
  }

  // For production, use Railway backend URL (deployed January 3, 2026)
  // Note: process.env is not available in browser, so we hardcode the production URL
  return 'https://virtuous-creativity-production.up.railway.app';
};

const BACKEND_URL = getBackendUrl();

/**
 * Custom error class for API errors
 */
class ApiError extends Error {
  constructor(
    public statusCode: number,
    public error: string,
    public detail?: string,
    public type?: ChatError['type']
  ) {
    super(error);
    this.name = 'ApiError';
  }
}

/**
 * Make HTTP request with error handling
 */
async function fetchWithErrorHandling<T>(
  url: string,
  options?: RequestInit
): Promise<T> {
  try {
    const response = await fetch(url, {
      ...options,
      headers: {
        'Content-Type': 'application/json',
        ...options?.headers,
      },
    });

    if (!response.ok) {
      // Try to parse error response
      let errorData: ChatError;
      try {
        errorData = await response.json();
      } catch {
        errorData = {
          error: `HTTP ${response.status}: ${response.statusText}`,
          status_code: response.status,
          type: 'unknown',
        };
      }

      throw new ApiError(
        response.status,
        errorData.error || 'Unknown error',
        errorData.detail,
        errorData.type
      );
    }

    return await response.json();
  } catch (error) {
    if (error instanceof ApiError) {
      throw error;
    }

    // Network error or other issue
    throw new ApiError(
      0,
      'Network error',
      error instanceof Error ? error.message : 'Failed to connect to backend',
      'service_error'
    );
  }
}

/**
 * Health check endpoint
 * GET /api/health
 */
export async function checkHealth(): Promise<HealthCheckResponse> {
  return fetchWithErrorHandling<HealthCheckResponse>(
    `${BACKEND_URL}/api/health`,
    { method: 'GET' }
  );
}

/**
 * Chat query endpoint
 * POST /api/chat/query
 */
export async function sendChatQuery(
  request: ChatQueryRequest
): Promise<ChatQueryResponse> {
  // Validate request before sending
  if (!request.query || request.query.trim().length === 0) {
    throw new ApiError(
      400,
      'Validation error',
      'Query cannot be empty',
      'validation_error'
    );
  }

  if (request.query.length > 2000) {
    throw new ApiError(
      400,
      'Validation error',
      'Query cannot exceed 2000 characters',
      'validation_error'
    );
  }

  // Limit conversation history to last 5 messages
  const limitedRequest: ChatQueryRequest = {
    ...request,
    conversation_history: request.conversation_history.slice(-5),
  };

  return fetchWithErrorHandling<ChatQueryResponse>(
    `${BACKEND_URL}/api/chat/query`,
    {
      method: 'POST',
      body: JSON.stringify(limitedRequest),
    }
  );
}

/**
 * Convert ApiError to ChatError for UI display
 */
export function apiErrorToChatError(error: unknown): ChatError {
  if (error instanceof ApiError) {
    return {
      error: error.error,
      detail: error.detail,
      type: error.type,
      status_code: error.statusCode,
    };
  }

  if (error instanceof Error) {
    return {
      error: 'Unknown error',
      detail: error.message,
      type: 'unknown',
    };
  }

  return {
    error: 'Unknown error',
    detail: 'An unexpected error occurred',
    type: 'unknown',
  };
}

/**
 * Check if backend is reachable
 */
export async function isBackendReachable(): Promise<boolean> {
  try {
    await checkHealth();
    return true;
  } catch {
    return false;
  }
}

export { ApiError };
