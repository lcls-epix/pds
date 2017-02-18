
with open('repos.txt', 'r') as fh:
    lines = fh.readlines()

with open('repos2.txt', 'w') as fh:
    for line in lines:
        line = line.rstrip()
        if line.startswith('#'):
            continue
        words = line.split()
        if words:
            name = words[0]
            print(name)
            fh.write(name + '\n')
