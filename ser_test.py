import sys
import asyncio

async def print_hello():
    while True:
        print('Hello')
        await asyncio.sleep(1)

async def print_from_stdin(q):
    while True:
        fut = asyncio.ensure_future(q.get())
        await fut
        print("got: " + fut.result())

def read_stdin(q):
    asyncio.ensure_future(q.put(sys.stdin.readline()))

q = asyncio.Queue()
loop = asyncio.get_event_loop()
loop.add_reader(sys.stdin, read_stdin, q)
loop.create_task(print_hello())
loop.create_task(print_from_stdin(q))

print('Started!')

try:
    loop.run_forever()
except KeyboardInterrupt:
    pass

loop.close()