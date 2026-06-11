June 9th & 10th, 2026 Update #69
Changed the PWM range from 1000 - 2000us to 600 - 2400us because the team realized that this servo motor accepts 500 - 2500us input and its range is 180 +/- 10 deg (we went with a safe margin of error)
Implement a flight-loop framework that measures IMU latency, control-loop timing, and flight-data logging without any delay. The system runs a synchronous control loop where sensor, state estimation, PID control, and actuation are executed in a fixed-rate scheduler. Logging operations (SD write and CSV export) are conditionally embedded in the same execution flow to measure their effect on real-time performance. The design isolates timing-critical control from diagnostic logging at the software level, while quantifying interference via per-cycle timing metrics (dt, slip, missed ticks, IMU read time, and SD write time in Yoosueng's code). Also we avoid dynamic memory allocation (no use of malloc entirely)

Basic Overview of Dual Processing in this Code
What is a Queue? In coding, a queue is an ordered sequence of tasks, jobs, or data waiting in a line to be processed by a computer or system. 

What is a record? A record is a data structure that can hold different data types. When running, a record is always in one of these four places 1). freeQueue (empty) 2). Core 0 (being filled) 3). filledQueue (waiting for logging in SD) 4). Core (being logged into SD)

Queues DO NOT hold data, they are permission slips to access data. 
freeQueue = permission to write 
filledQueue = permission to log 
Thus … 
filledQueue basically records ready for logging of records
freeQueue basically gets available empty slots for records

Core 0 Cycle: Does it need to log a new record? 
1). Get a free slot >>> freeQueue.get(...) 
2). Fill the record with snapshot >>> rec->rollCfDeg, rec->pitchCfDeg, rec->uRollDeg
3). Send filledQueue >>> filledQueue.put(rec) 

Core 1 Cycle: Is the logger told to wake up?
The core is sleeping >>> osEvent evt = filledQueue.get(); 


1). Take the pointer stored inside the queue message and treat it as a ControlRecord pointer 

2). Write snapshots to the SD card

3). Returns ownership back to Core 0.


At SD_LOG_HZ = 20 HZ, Core 1 consumes one record every 50 milliseconds; the queue gives Core 0 a full second of headroom before it would ever block.

Mbed RTOS Queue stores T* (which is a pointer), not T by its value. So Core 0 maintains a ring of buffer of control record slots  

recordPool
ControlRecord creates 20 record of RAM
For example …
recordPool[0]
recordPool[1]
recordPool[2]
...
recordPool[19] 
… where each is an empty slot that can hold one snapshot.

Queue depth is set to 20 records (QUEUE_DEPTH = 20). The record is 88 bytes (sizeof(ControlRecord) = 88). That means the queue is worth 1.76 kilobytes of RAM (20 x 80 = 1760), that's quite a line. At 20Hz of logging at 50ms per record, thats around 1 second of backlog tolerance, meaning that if the SD card freezes for one second, Core 0 can still keep going. Only after the queue fills Core 0 would have to wait. 







