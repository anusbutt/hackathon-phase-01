/**
 * ChatInput component - Input field for user queries
 * Handles text input, selected text display, and send button
 */

import React, { useState, useRef, useEffect } from 'react';
import { CHAT_CONSTANTS } from '../types/chat';
import styles from './ChatInput.module.css';

interface ChatInputProps {
  onSendMessage: (query: string, selectedText?: string | null) => Promise<void>;
  isLoading: boolean;
  selectedText?: string | null;
  onClearSelection?: () => void;
}

export const ChatInput: React.FC<ChatInputProps> = ({
  onSendMessage,
  isLoading,
  selectedText,
  onClearSelection,
}) => {
  const [query, setQuery] = useState('');
  const [charCount, setCharCount] = useState(0);
  const textareaRef = useRef<HTMLTextAreaElement>(null);

  // Auto-resize textarea
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
      textareaRef.current.style.height = `${textareaRef.current.scrollHeight}px`;
    }
  }, [query]);

  // Focus input on mount
  useEffect(() => {
    textareaRef.current?.focus();
  }, []);

  const handleInputChange = (e: React.ChangeEvent<HTMLTextAreaElement>) => {
    const value = e.target.value;
    if (value.length <= CHAT_CONSTANTS.MAX_QUERY_LENGTH) {
      setQuery(value);
      setCharCount(value.length);
    }
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    const trimmedQuery = query.trim();
    if (!trimmedQuery || isLoading) return;

    // Send message
    await onSendMessage(trimmedQuery, selectedText);

    // Clear input
    setQuery('');
    setCharCount(0);

    // Clear selection if provided
    if (selectedText && onClearSelection) {
      onClearSelection();
    }

    // Reset textarea height
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto';
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent<HTMLTextAreaElement>) => {
    // Submit on Enter (without Shift)
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSubmit(e);
    }
  };

  const isDisabled = isLoading || charCount === 0;

  return (
    <div className={styles.chatInput}>
      {/* Selected text indicator */}
      {selectedText && (
        <div className={styles.selectedTextBanner}>
          <div className={styles.selectedTextContent}>
            <strong>Explaining selected text:</strong>
            <span className={styles.selectedTextPreview}>
              {selectedText.length > 100
                ? `${selectedText.substring(0, 100)}...`
                : selectedText}
            </span>
          </div>
          {onClearSelection && (
            <button
              type="button"
              onClick={onClearSelection}
              className={styles.clearSelectionButton}
              aria-label="Clear selection"
            >
              ✕
            </button>
          )}
        </div>
      )}

      {/* Input form */}
      <form onSubmit={handleSubmit} className={styles.inputForm}>
        <div className={styles.inputWrapper}>
          <textarea
            ref={textareaRef}
            value={query}
            onChange={handleInputChange}
            onKeyDown={handleKeyDown}
            placeholder={
              selectedText
                ? 'Ask a question about the selected text...'
                : 'Ask a question about Physical AI & Humanoid Robotics...'
            }
            className={styles.textarea}
            disabled={isLoading}
            rows={1}
            aria-label="Chat query input"
          />

          <button
            type="submit"
            disabled={isDisabled}
            className={`${styles.sendButton} ${isDisabled ? styles.sendButtonDisabled : ''}`}
            aria-label="Send message"
          >
            {isLoading ? (
              <span className={styles.spinner}>⏳</span>
            ) : (
              <span className={styles.sendIcon}>➤</span>
            )}
          </button>
        </div>

        {/* Character counter */}
        <div className={styles.inputFooter}>
          <span
            className={`${styles.charCounter} ${
              charCount > CHAT_CONSTANTS.MAX_QUERY_LENGTH * 0.9
                ? styles.charCounterWarning
                : ''
            }`}
          >
            {charCount} / {CHAT_CONSTANTS.MAX_QUERY_LENGTH}
          </span>
          <span className={styles.hint}>
            Press Enter to send, Shift+Enter for new line
          </span>
        </div>
      </form>
    </div>
  );
};
