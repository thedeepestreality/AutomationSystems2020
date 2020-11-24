from fastapi import BackgroundTasks, FastAPI
from hypercorn.config import Config
from hypercorn.asyncio import serve

import asyncio
async def print_test(q):
    counter = 0
    while True:
        asyncio.ensure_future(q.put(counter))
        counter += 1
        await asyncio.sleep(5)

app = FastAPI()

@app.get("/")
async def read_root():
    fut = asyncio.ensure_future(q.get())
    await fut
    return {"Hello": fut.result(), "size" : q.qsize()}


@app.get("/items/{item_id}")
def read_item(item_id: int, q: str = None):
    return {"item_id": item_id, "q": q}

async def handle_echo(reader, writer):
    data = await reader.read(100)
    message = data.decode()
    addr = writer.get_extra_info('peername')
    print("Received %r from %r" % (message, addr))

    print("Send: %r" % message)
    writer.write(data)
    await writer.drain()

    print("Close the client socket")
    writer.close()
    
config = Config()
q = asyncio.Queue()
loop = asyncio.get_event_loop()
loop.create_task(serve(app, config)) # run fastapi
loop.create_task(print_test(q))
coro = asyncio.start_server(handle_echo, '127.0.0.1', 8888, loop=loop)
server = loop.run_until_complete(coro)

try:
    loop.run_forever()
except KeyboardInterrupt:
    pass

server.close()
loop.run_until_complete(server.wait_closed())
loop.close()
    
# @app.on_event("startup")
# async def startup_event(background_tasks: BackgroundTasks):
#     background_tasks.add_task(print_test)

# @app.get("/start")
# async def start_loop(background_tasks: BackgroundTasks):
#     background_tasks.add_task(print_test)