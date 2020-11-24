import asyncio

async def read_socket(reader):
    print("read socket")
    while True:
        message = await reader.readline()
        # message = data.decode()
        print("Send: %r" % message)

async def write_socket(writer):
    print("write socket")
    while True:
        writer.write("hello\n".encode('utf-8'))
        await writer.drain()
        await asyncio.sleep(3)

async def handle_echo(reader, writer):
    print("New connection")
    loop.create_task(read_socket(reader))
    loop.create_task(write_socket(writer))

loop = asyncio.get_event_loop()
coro = asyncio.start_server(handle_echo, '127.0.0.1', 8888, loop=loop)
server = loop.run_until_complete(coro)

# Serve requests until Ctrl+C is pressed
print('Serving on {}'.format(server.sockets[0].getsockname()))
try:
    loop.run_forever()
except KeyboardInterrupt:
    pass

# Close the server
server.close()
loop.run_until_complete(server.wait_closed())
loop.close()