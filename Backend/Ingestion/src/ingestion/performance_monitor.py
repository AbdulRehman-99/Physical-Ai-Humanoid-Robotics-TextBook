"""Module for performance monitoring of the ingestion pipeline."""

import time
import threading
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass
from datetime import datetime
import logging
from collections import defaultdict


@dataclass
class PerformanceMetric:
    """Data class to store performance metrics."""
    name: str
    value: float
    unit: str
    timestamp: datetime
    tags: Dict[str, str]


class PerformanceMonitor:
    """Class to monitor and track performance metrics of the ingestion pipeline."""

    def __init__(self):
        """Initialize the performance monitor."""
        self.metrics: List[PerformanceMetric] = []
        self.lock = threading.Lock()
        self.logger = logging.getLogger(__name__)

        # Track pipeline statistics
        self.pipeline_stats = {
            'total_processed': 0,
            'total_failed': 0,
            'total_chunks_created': 0,
            'total_processing_time': 0.0,
            'average_processing_time': 0.0
        }

    def start_timer(self) -> float:
        """Start a timer and return the start time."""
        return time.time()

    def stop_timer(self, start_time: float) -> float:
        """Stop a timer and return the elapsed time in seconds."""
        return time.time() - start_time

    def record_metric(self, name: str, value: float, unit: str = '', tags: Optional[Dict[str, str]] = None) -> None:
        """Record a performance metric."""
        with self.lock:
            metric = PerformanceMetric(
                name=name,
                value=value,
                unit=unit,
                timestamp=datetime.now(),
                tags=tags or {}
            )
            self.metrics.append(metric)

    def record_pipeline_step(self, step_name: str, duration: float, success: bool = True) -> None:
        """Record the duration of a pipeline step."""
        status = 'success' if success else 'failed'
        tags = {'step': step_name, 'status': status}
        self.record_metric(f'pipeline.{step_name}.duration', duration, 'seconds', tags)

        if success:
            self.pipeline_stats['total_processed'] += 1
        else:
            self.pipeline_stats['total_failed'] += 1

        self.logger.debug(f"Pipeline step '{step_name}' completed in {duration:.3f}s with status: {status}")

    def record_document_processing(self, duration: float, success: bool = True, chunks_created: int = 0) -> None:
        """Record metrics for document processing."""
        with self.lock:
            if success:
                self.pipeline_stats['total_processed'] += 1
                self.pipeline_stats['total_chunks_created'] += chunks_created
                self.pipeline_stats['total_processing_time'] += duration
                self.pipeline_stats['average_processing_time'] = (
                    self.pipeline_stats['total_processing_time'] / self.pipeline_stats['total_processed']
                )

        self.record_metric('document.processing.duration', duration, 'seconds', {'success': str(success)})
        if chunks_created > 0:
            self.record_metric('document.chunks.created', chunks_created, 'count')

    def get_metrics_summary(self) -> Dict[str, Any]:
        """Get a summary of all recorded metrics."""
        with self.lock:
            # Calculate summary statistics
            if not self.metrics:
                return {}

            # Group metrics by name
            metrics_by_name = defaultdict(list)
            for metric in self.metrics:
                metrics_by_name[metric.name].append(metric)

            summary = {}
            for name, metrics in metrics_by_name.items():
                values = [m.value for m in metrics]
                summary[name] = {
                    'count': len(values),
                    'total': sum(values),
                    'average': sum(values) / len(values),
                    'min': min(values),
                    'max': max(values),
                    'unit': metrics[0].unit
                }

            # Add pipeline statistics
            summary['pipeline_stats'] = self.pipeline_stats.copy()

            return summary

    def get_recent_metrics(self, minutes: int = 5) -> List[PerformanceMetric]:
        """Get metrics recorded in the last N minutes."""
        cutoff_time = datetime.now()
        cutoff_time = cutoff_time.replace(minute=cutoff_time.minute - minutes)

        with self.lock:
            recent_metrics = [m for m in self.metrics if m.timestamp >= cutoff_time]
            return recent_metrics

    def reset_metrics(self) -> None:
        """Reset all recorded metrics."""
        with self.lock:
            self.metrics.clear()
            # Reset pipeline stats but keep the structure
            for key in self.pipeline_stats:
                if isinstance(self.pipeline_stats[key], (int, float)):
                    self.pipeline_stats[key] = 0 if isinstance(self.pipeline_stats[key], int) else 0.0

    def log_performance_report(self) -> None:
        """Log a performance report with current metrics."""
        summary = self.get_metrics_summary()

        self.logger.info("=" * 60)
        self.logger.info("PERFORMANCE REPORT")
        self.logger.info("=" * 60)

        for name, stats in summary.items():
            if name == 'pipeline_stats':
                self.logger.info("Pipeline Statistics:")
                for stat_name, value in stats.items():
                    self.logger.info(f"  {stat_name}: {value}")
            else:
                self.logger.info(f"{name}: avg={stats['average']:.3f}, min={stats['min']:.3f}, max={stats['max']:.3f} {stats['unit']}")

        self.logger.info("=" * 60)


# Global performance monitor instance
monitor = PerformanceMonitor()


def monitor_pipeline_step(step_name: str):
    """Decorator to monitor the performance of a pipeline step."""
    def decorator(func: Callable) -> Callable:
        def wrapper(*args, **kwargs):
            start_time = monitor.start_timer()
            try:
                result = func(*args, **kwargs)
                duration = monitor.stop_timer(start_time)
                monitor.record_pipeline_step(step_name, duration, success=True)
                return result
            except Exception as e:
                duration = monitor.stop_timer(start_time)
                monitor.record_pipeline_step(step_name, duration, success=False)
                raise e
        return wrapper
    return decorator


def measure_function_performance(func_name: str = None):
    """Decorator to measure the performance of any function."""
    def decorator(func: Callable) -> Callable:
        nonlocal func_name
        if not func_name:
            func_name = func.__name__

        def wrapper(*args, **kwargs):
            start_time = monitor.start_timer()
            try:
                result = func(*args, **kwargs)
                duration = monitor.stop_timer(start_time)
                monitor.record_metric(f'function.{func_name}.duration', duration, 'seconds')
                return result
            except Exception as e:
                duration = monitor.stop_timer(start_time)
                monitor.record_metric(f'function.{func_name}.duration.error', duration, 'seconds')
                raise e
        return wrapper
    return decorator


def get_global_monitor() -> PerformanceMonitor:
    """Get the global performance monitor instance."""
    return monitor