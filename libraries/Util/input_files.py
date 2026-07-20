import re

##Matches C++'s Datalogger::ImportFile: every line of the file becomes one
##row (comments/blank lines included), parsed like atof() -- leading numeric
##token only, 0.0 if nothing numeric is found. Same convention Config.txt
##and Simulation.txt already use ("<value>\t!comment").
_LEADING_FLOAT = re.compile(r'^\s*[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?')


def _atof(line):
    m = _LEADING_FLOAT.match(line)
    return float(m.group(0)) if m else 0.0


def read_input_file(path):
    """Read a Config.txt/Simulation.txt style file into a flat list of floats.

    Row N (1-indexed, matching the C++ MATLAB.get(row,1) convention used
    throughout this codebase) is data[N-1].
    """
    with open(path, 'r') as f:
        return [_atof(line) for line in f]


if __name__ == '__main__':
    import tempfile, os
    sample = "0.1 \t!Standard Out Print Rate\n1.0\t!Something\n!Just a comment\n\n5\n"
    fd, tmp = tempfile.mkstemp()
    os.write(fd, sample.encode())
    os.close(fd)
    data = read_input_file(tmp)
    os.remove(tmp)
    assert data == [0.1, 1.0, 0.0, 0.0, 5.0], data
    print('input_files.py OK:', data)
