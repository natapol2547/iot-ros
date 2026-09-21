"""The browser WebSocket accepts a first message that comes after a heartbeat.

Regression test: with permessage-deflate negotiated, aiohttp 3.14 closed the socket with
1002 (protocol error) when a client's first frame was a pong and its next one a
compressed message. That is a visitor who watched the page past the first heartbeat and
then pressed STOP: the E-stop was lost.
"""

import asyncio
import json

import aiohttp
from aiohttp import web

from iot_robot_web.web_controller import websocket_response

HEARTBEAT = 0.2  # s, shortened from the node's 5 s to keep the test fast


async def message_after_idle(idle):
    """Connect like a browser, stay silent for `idle` s, send one message.

    Returns (messages the server received, close code seen by the client or None).
    """
    received = []

    async def handler(request):
        ws = websocket_response(heartbeat=HEARTBEAT)
        await ws.prepare(request)
        async for msg in ws:
            if msg.type == aiohttp.WSMsgType.TEXT:
                received.append(json.loads(msg.data))
                await ws.send_str(msg.data)
        return ws

    app = web.Application()
    app.router.add_get("/ws", handler)
    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "127.0.0.1", 0)
    await site.start()
    port = runner.addresses[0][1]
    replies = asyncio.Queue()
    try:
        async with aiohttp.ClientSession() as session:
            # compress=15 offers permessage-deflate, as every browser does
            async with session.ws_connect(f"http://127.0.0.1:{port}/ws", compress=15) as ws:

                async def read():
                    # Reading is what answers the server's pings (autoping)
                    async for msg in ws:
                        await replies.put(msg)
                    await replies.put(None)

                reader = asyncio.create_task(read())
                await asyncio.sleep(idle)
                await ws.send_json({"type": "estop", "engaged": True})
                try:
                    reply = await asyncio.wait_for(replies.get(), 2.0)
                except asyncio.TimeoutError:
                    reply = None
                close_code = ws.close_code if reply is None else None
                reader.cancel()
    finally:
        await runner.cleanup()
    return received, close_code


def test_first_message_after_heartbeats_is_received():
    received, close_code = asyncio.run(message_after_idle(idle=5 * HEARTBEAT))
    assert close_code is None
    assert received == [{"type": "estop", "engaged": True}]


def test_immediate_first_message_is_received():
    received, close_code = asyncio.run(message_after_idle(idle=0.0))
    assert close_code is None
    assert received == [{"type": "estop", "engaged": True}]
