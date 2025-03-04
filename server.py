import asyncio
import concurrent.futures
import struct
import time

from rtde_control import RTDEControlInterface

torques = [0] * 6
executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)


def comm_robot():
    rtde_c = RTDEControlInterface("169.254.9.43")
    print("Connected to UR5")
    while True:
        global torques
        torques = rtde_c.getJointTorques()
        time.sleep(0.01)


async def comm_hololens():
    async def callback(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
        print("Connected to HoloLens")
        while True:
            try:
                await reader.read(1024)
                writer.write(struct.pack("!" + "f" * 6, *torques))
                await writer.drain()
            except:
                break


    await asyncio.start_server(callback, host="0.0.0.0", port=21200)


async def main():
    loop = asyncio.get_running_loop()
    await asyncio.gather(
        comm_hololens(),
        loop.run_in_executor(executor, comm_robot)
    )


if __name__ == "__main__":
    asyncio.run(main())
