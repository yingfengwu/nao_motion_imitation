import sys
import subprocess
import argparse


def main():
    # Command line arguments
    print(sys.path)
    sys.path.extend(['F:/wyf/Python_Code/Bair_paper_code/nao_motion_imitation-master/venv/Lib/site-packages'])
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("-n", dest="n", type=int, default='10')

    arg = arg_parser.parse_args()
    assert (arg.n > 0)

    print('Running with {:d} workers'.format(arg.n))
    cmd = 'mpiexec -n {:d} F:/wyf/Python_Code/Bair_paper_code/nao_motion_imitation-master/venv/Scripts/python run.py '.format(arg.n)
    print('cmd: ' + cmd)
    subprocess.call(cmd, shell=True)
    return


if __name__ == '__main__':
    main()