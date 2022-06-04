import sys
import subprocess
import argparse


def main():
    # Command line arguments
    print(sys.path)
    sys.path.extend(['E:/my_data/MachineLearning/Python_Code/Bair_paper_code/motion_imitation-revised/venv/Scripts'])
    arg_parser = argparse.ArgumentParser()
    arg_parser.add_argument("-n", dest="n", type=int, default='3')

    arg = arg_parser.parse_args()
    assert (arg.n > 0)

    print('Running with {:d} workers'.format(arg.n))
    cmd = 'mpiexec -n {:d} E:/my_data/MachineLearning/Python_Code/Bair_paper_code/motion_imitation-revised/venv/Scripts/python run.py '.format(arg.n)
    print('cmd: ' + cmd)
    subprocess.call(cmd, shell=True)
    return


if __name__ == '__main__':
    main()