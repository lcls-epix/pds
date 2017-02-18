with open('authors.txt', 'r') as fh:
    lines = fh.readlines()

with open('authors2.txt', 'w') as fh:
    for line in lines:
        index = line.find('@')
        author = line[:index]
        index = line.find('=')
        email = line[:index]
        fh.write('%s %s <%s>\n' %(line.strip(), author, email.strip()))
