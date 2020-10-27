import sys
import asyncio
import aioserial

async def read_and_print(aioserial_instance: aioserial.AioSerial):
    while True:
        print((await aioserial_instance.read_async(2)).decode(errors='ignore'), end='\n', flush=True)

async def print_from_stdin(q, ser):
    while True:
        fut = asyncio.ensure_future(q.get())
        await fut
        str = fut.result()
        print("got: " + str)
        await ser.write_async(bytearray(str,'utf-8'))

def read_stdin(q):
    asyncio.ensure_future(q.put(sys.stdin.readline()))

ser = aioserial.AioSerial(port='/dev/ttyUSB1',baudrate=9600,timeout=2)
print('Port opened')
q = asyncio.Queue()
loop = asyncio.get_event_loop()
loop.add_reader(sys.stdin, read_stdin, q)
loop.create_task(print_from_stdin(q, ser))
loop.create_task(read_and_print(ser))

print('Started!')

try:
    loop.run_forever()
except KeyboardInterrupt:
    pass

ser.close()
loop.close()
