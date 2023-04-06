FILE=values.txt
if test -f "$FILE"; then
    rm -rf values.txt
fi


python3 qpSolve.py --numCables 2 --runs 100
python3 qpSolve.py --numCables 4 --runs 100
python3 qpSolve.py --numCables 8 --runs 100
python3 qpSolve.py --numCables 16 --runs 100
python3 qpSolve.py --numCables 32 --runs 100
# python3 qpSolve.py --numCables 64 --runs 100


python3 plot.py --filename values.txt