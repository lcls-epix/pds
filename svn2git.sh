#!/bin/bash

DIR=$HOME/svn
TOKEN=

function convert_repo {
    cd $DIR
    #prepare authors files
    svn log https://pswww.slac.stanford.edu/svn-readonly/psdmrepo/$1/ \
            --xml --quiet | grep author | sort -u | \
            perl -pe 's/.*>(.*?)<.*/$1 = /' > authors.txt

    python3 authors.py

    git svn clone https://pswww.slac.stanford.edu/svn-readonly/psdmrepo/$1 \
                  --stdlayout --no-metadata \
                  --authors-file=authors2.txt --prefix "" repos/$1

    cd $DIR/repos/$1

    # convert svn tag branches to git tags
    for t in $(git for-each-ref --format='%(refname:short)' refs/remotes/tags); do \
        git tag ${t/tags\//} $t && git branch -D -r $t; done

    for b in $(git for-each-ref --format='%(refname:short)' refs/remotes); do \
        git branch $b refs/remotes/$b && git branch -D -r $b; done

    git branch -d trunk
}

function push_repo {
    # create repository on github
    curl -u weninc:$TOKEN \
         https://api.github.com/orgs/lcls-psana/repos \
         -d "{\"name\": \"$1\"}"

    cd $DIR/repos/$1
    git remote add origin git@github.com:lcls-psana/$1.git

    git push origin --all
    git push origin --tags
}

function delete_repo {
    curl -X DELETE -u weninc:$TOKEN  \
          https://api.github.com/repos/lcls-psana/$1
}

while read repo; do
    echo $repo
    convert_repo $repo
    push_repo $repo
done < repos2.txt
