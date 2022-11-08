import math
import numpy as np
from Queue import Queue
if __name__ == '__main__':
    q = Queue(10)
    for i in range(100):
        if not q.full():
            q.put([i]*2)
        else:
            q.get()
            q.put([i]*2)
            print q.queue
    a = np.array(q.queue)
    print np.average(q.queue,axis=0)-[1.,1.]
    print math.atan2(np.sqrt(3),3)*180/np.pi