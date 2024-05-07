import threading
import time


# Create a flag that can be shared between threads
stop_flag = threading.Event()


def run_task(stop_flag):
    # This is the task that you want to run in the background
    print("Task started")
    for i in range(1000):
        if stop_flag.is_set():
            print("Stop get back !!")
            break
        print(f"Task iteration {i}")
        time.sleep(1) # Simulate work being done
    else:
        print("Task finished")

def change_flag(stop_flag):
    # This thread will change the flag after 80 seconds
    time.sleep(80)
    print("Changing flag")
    stop_flag.set()

# Start the task in a separate thread
thread_task = threading.Thread(target=run_task, args=(stop_flag,))
thread_task.start()

# Start the thread that will change the flag after 80 seconds
thread_flag = threading.Thread(target=change_flag, args=(stop_flag,))
thread_flag.start()

# Wait for the task to finish
thread_task.join()

# If the task was stopped before it finished, we need to handle the exception
if stop_flag.is_set():
    print("Task was stopped")
else:
    print("Task finished naturally")
