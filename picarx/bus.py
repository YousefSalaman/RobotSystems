import time
import typing
import functools
import concurrent.futures
from dataclasses import dataclass

from readerwriterlock import rwlock


def forever(time_delay):
    """Decorator to run the function forever"""

    @functools.wraps
    def forever_decorator(func):

        def forever_wrapper(*args):

            while True:
                func(*args)
                time.sleep(time_delay)

        return forever_wrapper
    
    return forever_decorator


@dataclass(slots=True)
class ConcurrentHelper:
    """Data class that represents either a consumer or a producer"""

    f: typing.Callable
    time_delay: typing.Union[int , float]


class Bus:
    """Bus that connects two different parts of the code using a
    shared memory space and threads to execute multiple tasks
    concurrently."""

    def __init__(self,
        consumer: typing.Optional[ConcurrentHelper] = None,
        producer: typing.Optional[ConcurrentHelper] = None
        ):

        self.message = None
        self._consumer = consumer
        self._producer = producer
        self.lock = rwlock.RWLockWriteD()

    def write(self,
        message: typing.Any
        ):
        """Save a value in the shared storage space"""

        with self.lock.gen_wlock():
            self.message = message

    def read(self):
        """Read a value in the shared storage space"""

        with self.lock.gen_rlock():
            return self.message

    def run(self):
        """Create thread pools to execute functions in a possible consumer-producer pair"""

        # Get the concurrent helpers that were defined
        helpers = [helper for helper in (self.consumer, self.producer) if helper is not None]
    
        # Create executors from the concurrent helpers
        executors = []
        with concurrent.futures.ThreadPoolExecutor(max_workers=len(helpers)) as executor:
            for helper in helpers:
                executors.append(executor.submit(forever(helper.time_delay)(helper.f), self.message))
        
        # Display any errors in the code
        for executor in executors:
            print(executor.result())
