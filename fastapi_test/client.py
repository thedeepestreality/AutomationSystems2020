import asyncio
import sys

reader = None
writer = None

async def read_from_server(reader):
    print("read_from_server")
    while True:
        data = await reader.readline()
        print('Received: %r' % data)

async def tcp_echo_client(q, loop):
    reader, writer = await asyncio.open_connection('127.0.0.1', 8888,
                                                   loop=loop)

    loop.create_task(print_from_stdin(q, writer))
    loop.create_task(read_from_server(reader))

async def print_from_stdin(q, writer):
    print("print_form_stdin")
    while True:
        fut = asyncio.ensure_future(q.get())
        await fut
        str = fut.result()
        print("got: " + str)
        writer.write(str.encode('utf-8'))
        await writer.drain()

def read_stdin(q):
    asyncio.ensure_future(q.put(sys.stdin.readline()))

q = asyncio.Queue()
loop = asyncio.get_event_loop()
loop.add_reader(sys.stdin, read_stdin, q)
loop.run_until_complete(tcp_echo_client(q, loop))

try:
    loop.run_forever()
except KeyboardInterrupt:
    pass
loop.close()