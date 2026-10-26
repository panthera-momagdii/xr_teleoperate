"""RT5c: run the launcher with stdin forced back to BLOCKING every 0.5 s (the RT5b bug), to prove a
keyboard listener that cannot stop is non-fatal. Test only."""
import fcntl, os, runpy, sys, threading, time
def clear():
    while True:
        try:
            fl = fcntl.fcntl(0, fcntl.F_GETFL); fcntl.fcntl(0, fcntl.F_SETFL, fl & ~os.O_NONBLOCK)
        except Exception: pass
        time.sleep(0.5)
threading.Thread(target=clear, daemon=True).start()
sys.argv = sys.argv[1:]; runpy.run_path(sys.argv[0], run_name="__main__")
