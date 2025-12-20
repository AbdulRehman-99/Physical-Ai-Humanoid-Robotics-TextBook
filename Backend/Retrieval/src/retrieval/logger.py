import logging
import json
from datetime import datetime
from typing import Any, Dict, List
import os

class RetrievalLogger:
    """
    Logging infrastructure for retrieval operations, focused on accuracy checks and performance metrics.
    """

    def __init__(self, log_file: str = "retrieval_logs.json"):
        self.log_file = log_file
        self.logger = logging.getLogger("retrieval")
        self.logger.setLevel(logging.INFO)

        # Create file handler for JSON logs
        if not self.logger.handlers:
            file_handler = logging.FileHandler(self.log_file)
            file_handler.setLevel(logging.INFO)
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            file_handler.setFormatter(formatter)
            self.logger.addHandler(file_handler)

    def log_retrieval_request(self, request_data: Dict[str, Any]) -> None:
        """
        Log a retrieval request with all relevant information.

        Args:
            request_data: Dictionary containing request information
        """
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "event_type": "retrieval_request",
            "data": request_data
        }
        self._write_json_log(log_entry)
        self.logger.info(f"Retrieval request: {request_data['query_text']}")

    def log_retrieval_response(self, response_data: Dict[str, Any]) -> None:
        """
        Log a retrieval response with all relevant information.

        Args:
            response_data: Dictionary containing response information
        """
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "event_type": "retrieval_response",
            "data": response_data
        }
        self._write_json_log(log_entry)
        num_results = response_data.get('num_results', 0)
        self.logger.info(f"Retrieval response: {num_results} results returned")

    def log_validation_result(self, validation_data: Dict[str, Any]) -> None:
        """
        Log a validation result for accuracy checking.

        Args:
            validation_data: Dictionary containing validation information
        """
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "event_type": "validation_result",
            "data": validation_data
        }
        self._write_json_log(log_entry)
        self.logger.info(f"Validation result: relevance_score={validation_data.get('relevance_score')}")

    def log_performance_metrics(self, metrics_data: Dict[str, Any]) -> None:
        """
        Log performance metrics for monitoring.

        Args:
            metrics_data: Dictionary containing performance metrics
        """
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "event_type": "performance_metrics",
            "data": metrics_data
        }
        self._write_json_log(log_entry)
        self.logger.info(f"Performance metrics: latency={metrics_data.get('latency_ms')}ms")

    def _write_json_log(self, log_entry: Dict[str, Any]) -> None:
        """
        Write a log entry in JSON format to the log file.

        Args:
            log_entry: Dictionary containing the log entry
        """
        with open(self.log_file, 'a', encoding='utf-8') as f:
            f.write(json.dumps(log_entry) + '\n')

    def log_chunk_for_accuracy(self, chunk_data: Dict[str, Any]) -> None:
        """
        Log individual chunks for accuracy analysis.

        Args:
            chunk_data: Dictionary containing chunk information for analysis
        """
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "event_type": "chunk_for_accuracy",
            "data": chunk_data
        }
        self._write_json_log(log_entry)
        self.logger.info(f"Chunk logged for accuracy: {chunk_data.get('chunk_id')}")

    def log_retrieved_chunks(self, chunks: List[Dict[str, Any]]) -> None:
        """
        Log retrieved chunks for accuracy analysis.

        Args:
            chunks: List of chunk dictionaries to log
        """
        for chunk in chunks:
            self.log_chunk_for_accuracy(chunk)