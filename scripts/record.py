import subprocess
import struct
import time
import wave

ret = subprocess.run(["pactl", "list"], capture_output=True)
lines = ret.stdout.decode().split("\n")
lines = [l.strip() for l in lines]

app_found = False
looking_for_bin = False

for l in lines:
    if "Sink Input" in l:
        sink_id = int(l.split("#")[1])
        looking_for_bin = True
    elif looking_for_bin and "application.process.binary" in l:
        if l.split("\"")[1] == "sdr_pmr446":
            app_found = True
            break
        looking_for_bin = False

if app_found:
    print("The app sink id is {}".format(sink_id))
    command = ["pacat", "-r", "-n", "\"PMRRecording\"", "--channels", "1",
               "--rate", "12500", "--format=s32le", "--monitor-stream={}".format(sink_id)]
    print("Using 'pacat' command: \"{}\"".format(" ".join(command)))
    process = subprocess.Popen(
        command, stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.PIPE)

    with wave.open("output_{}.wav".format(int(time.time())), "w") as f:
        f.setnchannels(1)
        f.setsampwidth(4)
        f.setframerate(12500)

        data = bytearray([])

        while True:
            output = process.stdout.readline()

            if output == b'' and process.poll() is not None:
                break

            if output:
                data += output
                samples = [struct.unpack('<i', data[n:n+4])[0]
                           for n in range(0, 4 * (len(data) // 4), 4)]

                chunks = [samples[n:n+1000]
                          for n in range(0, len(samples), 1000)]

                chunks = filter(lambda ch: any(
                    map(lambda x: x != 0, ch)), chunks)

                bs = bytearray([])
                for ch in chunks:
                    for s in ch:
                        bs.extend(struct.pack("<i", s))

                f.writeframes(bs)

                if (len(data) % 4) != 0:
                    data = data[-(len(data) % 4):]
                else:
                    data = bytearray([])
            rc = process.poll()

        print("Recorder script exited")
        print("Check if app is still running")

else:
    print("App not found")
