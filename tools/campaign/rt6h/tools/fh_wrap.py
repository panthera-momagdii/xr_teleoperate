"""Run the launcher with faulthandler on SIGUSR1 (all threads' Python stacks to FH_OUT).
Test aid for the RT6h start-up hang; no change to the launcher."""
import faulthandler, os, runpy, signal, sys
_fh = open(os.environ.get("FH_OUT", "/dev/stderr"), "w")
faulthandler.register(signal.SIGUSR1, file=_fh, all_threads=True)
sys.argv = sys.argv[1:]
sys.path.insert(0, os.path.dirname(os.path.abspath(sys.argv[0])))
runpy.run_path(sys.argv[0], run_name="__main__")
