"""RT5c: reproduce RT5b's attach check exactly -- `tmux display-message` with the tty inherited --
a few seconds into the launcher, then let [q] show whether the exit still completes. Test only."""
import faulthandler, runpy, signal, subprocess, sys, threading, time
faulthandler.register(signal.SIGUSR1, file=open('/home/unitree/panthera/remote-teleop-20261012/rt5c/out/qtest/stacks_oldcheck.txt', 'w'), all_threads=True)
def later():
    time.sleep(12)
    subprocess.run(["tmux", "display-message", "-p", "#S"], capture_output=True, text=True, timeout=3)
threading.Thread(target=later, daemon=True).start()
sys.argv = sys.argv[1:]; runpy.run_path(sys.argv[0], run_name="__main__")
