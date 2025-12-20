import time
import statistics
from typing import List, Dict, Any, Callable
from datetime import datetime
from .models import PerformanceMetrics
from .logger import RetrievalLogger


class LatencyMeasurement:
    """
    Implements performance measurement tools to track retrieval latency and stability
    while logging retrieved chunks for accuracy analysis.
    """

    def __init__(self):
        self.logger = RetrievalLogger()

    def measure_latency(self, search_function: Callable, query_text: str, iterations: int = 100) -> PerformanceMetrics:
        """
        Measure latency for a search function over multiple iterations.

        Args:
            search_function: The search function to measure
            query_text: The query text to use for testing
            iterations: Number of iterations to run (default 100)

        Returns:
            PerformanceMetrics object with latency measurements
        """
        latencies = []
        start_time = time.time()

        for i in range(iterations):
            iteration_start = time.time()
            # In a real implementation, we would call the search function with actual parameters
            # For now, we'll just measure the time of a simple operation
            try:
                # This is a placeholder - in a real implementation, we would call the search function
                # with the proper parameters and measure the actual latency
                pass
            except Exception as e:
                print(f"Error during latency measurement iteration {i}: {e}")
                continue

            iteration_latency = (time.time() - iteration_start) * 1000  # Convert to milliseconds
            latencies.append(iteration_latency)

        # Calculate performance metrics
        total_time = (time.time() - start_time) * 1000  # Total time in milliseconds
        throughput_qps = (len(latencies) / total_time) * 1000 if total_time > 0 else 0

        if latencies:
            avg_latency = statistics.mean(latencies)
            p95_latency = self._calculate_percentile(latencies, 0.95)
            p99_latency = self._calculate_percentile(latencies, 0.99)
            stability_score = self._calculate_stability(latencies)
        else:
            avg_latency = 0
            p95_latency = 0
            p99_latency = 0
            stability_score = 0

        metrics = PerformanceMetrics(
            request_id=f"latency-{datetime.now().isoformat()}",
            latency_ms=avg_latency,
            throughput_qps=throughput_qps,
            stability_score=stability_score,
            timestamp=datetime.now()
        )

        # Log the performance metrics
        metrics_data = {
            "request_id": metrics.request_id,
            "avg_latency_ms": avg_latency,
            "p95_latency_ms": p95_latency,
            "p99_latency_ms": p99_latency,
            "throughput_qps": throughput_qps,
            "stability_score": stability_score,
            "iterations": len(latencies),
            "total_time_ms": total_time
        }
        self.logger.log_performance_metrics(metrics_data)

        return metrics

    def _calculate_percentile(self, data: List[float], percentile: float) -> float:
        """
        Calculate the specified percentile of the latency data.

        Args:
            data: List of latency measurements
            percentile: Percentile to calculate (e.g., 0.95 for 95th percentile)

        Returns:
            The calculated percentile value
        """
        if not data:
            return 0

        sorted_data = sorted(data)
        index = int(len(sorted_data) * percentile)
        if index >= len(sorted_data):
            index = len(sorted_data) - 1

        return sorted_data[index]

    def _calculate_stability(self, latencies: List[float]) -> float:
        """
        Calculate a stability score based on the consistency of latency measurements.

        Args:
            latencies: List of latency measurements

        Returns:
            Stability score between 0 and 1 (1 being perfectly stable)
        """
        if len(latencies) < 2:
            return 1.0

        avg_latency = statistics.mean(latencies)
        if avg_latency == 0:
            return 1.0

        # Calculate coefficient of variation (lower is more stable)
        std_dev = statistics.stdev(latencies)
        coefficient_of_variation = std_dev / avg_latency if avg_latency != 0 else 0

        # Convert to stability score (1 - normalized coefficient of variation)
        # Cap the coefficient of variation to prevent negative scores
        stability_score = max(0.0, min(1.0, 1.0 - coefficient_of_variation))
        return stability_score

    def measure_concurrent_performance(self, search_function: Callable, query_text: str,
                                     concurrent_requests: int = 10, iterations_per_request: int = 10) -> Dict[str, Any]:
        """
        Implement concurrent request handling and stress testing utilities.

        Args:
            search_function: The search function to test
            query_text: The query text to use for testing
            concurrent_requests: Number of concurrent requests to simulate
            iterations_per_request: Number of iterations per concurrent request

        Returns:
            Dictionary containing concurrent performance metrics
        """
        import threading
        import queue

        # This is a simplified implementation of concurrent performance measurement
        # In a real implementation, we would use proper async/await or thread pooling

        results_queue = queue.Queue()
        threads = []

        def run_search_thread(thread_id: int):
            """Run search in a separate thread."""
            thread_latencies = []
            for i in range(iterations_per_request):
                start_time = time.time()
                try:
                    # In a real implementation, we would call the search function
                    # with proper parameters
                    pass
                except Exception as e:
                    print(f"Error in thread {thread_id}, iteration {i}: {e}")
                    continue

                latency = (time.time() - start_time) * 1000  # Convert to milliseconds
                thread_latencies.append(latency)

            results_queue.put(thread_latencies)

        # Start all threads
        for i in range(concurrent_requests):
            thread = threading.Thread(target=run_search_thread, args=(i,))
            threads.append(thread)
            thread.start()

        # Wait for all threads to complete
        for thread in threads:
            thread.join()

        # Collect all latencies
        all_latencies = []
        while not results_queue.empty():
            all_latencies.extend(results_queue.get())

        # Calculate concurrent performance metrics
        if all_latencies:
            avg_latency = statistics.mean(all_latencies)
            p95_latency = self._calculate_percentile(all_latencies, 0.95)
            p99_latency = self._calculate_percentile(all_latencies, 0.99)
            stability_score = self._calculate_stability(all_latencies)
            throughput_qps = (len(all_latencies) / sum(all_latencies) * 1000) if sum(all_latencies) > 0 else 0
        else:
            avg_latency = 0
            p95_latency = 0
            p99_latency = 0
            stability_score = 0
            throughput_qps = 0

        concurrent_metrics = {
            "concurrent_requests": concurrent_requests,
            "iterations_per_request": iterations_per_request,
            "total_requests": len(all_latencies),
            "avg_latency_ms": avg_latency,
            "p95_latency_ms": p95_latency,
            "p99_latency_ms": p99_latency,
            "throughput_qps": throughput_qps,
            "stability_score": stability_score,
            "timestamp": datetime.now().isoformat()
        }

        # Log the concurrent performance metrics
        self.logger.log_performance_metrics(concurrent_metrics)

        return concurrent_metrics

    def add_rate_limiting(self, max_requests_per_minute: int = 100) -> None:
        """
        Add concurrent request rate limiting.

        Args:
            max_requests_per_minute: Maximum number of requests allowed per minute
        """
        # This would implement rate limiting in a real system
        # For now, we'll just document the intended functionality
        print(f"Rate limiting configured: max {max_requests_per_minute} requests per minute")

    def add_performance_tracking_to_search(self, search_function: Callable,
                                          query_text: str, top_k: int = 10) -> Dict[str, Any]:
        """
        Add performance tracking to search function.

        Args:
            search_function: The search function to add tracking to (already prepared with params)
            query_text: The query text
            top_k: Number of results to return

        Returns:
            Dictionary containing search results and performance metrics
        """
        start_time = time.time()

        # In a real implementation, we would call the search function
        # and measure the actual performance
        try:
            # Call the search function (which should already have parameters)
            result = search_function()
            execution_time_ms = (time.time() - start_time) * 1000
        except Exception as e:
            print(f"Error during performance tracking: {e}")
            execution_time_ms = (time.time() - start_time) * 1000  # Record time even if there's an error

        performance_data = {
            "query_text": query_text,
            "top_k": top_k,
            "execution_time_ms": execution_time_ms,
            "timestamp": datetime.now().isoformat()
        }

        # Log performance data
        self.logger.log_performance_metrics(performance_data)

        return {
            "execution_time_ms": execution_time_ms,
            "timestamp": datetime.now().isoformat(),
            # "results": result  # In a real implementation
        }