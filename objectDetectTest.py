from multiprocessing import Process
from multiprocessing import Queue
from objectDetector import *

def Consumer(queue):
    #    print('Consumer: Running', flush=True)
    # consume work
    while True:
        # get a unit of work
        item = queue.get()
        # check for stop
        #        if item is None:
        #            break
        # report
        print(f'>got {item}', flush=True)
    # all done
    print('Consumer: Done', flush=True)

# main:
queue = Queue()

# start the consumer
consumer_process = Process(target=Consumer, args=(queue,))
consumer_process.start()

# start the producer
producer_process = Process(target=ObjectTracker, args=(queue,))
producer_process.start()

# wait for all processes to finish
producer_process.join()
consumer_process.join()
