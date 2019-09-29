sudo git filter-branch --force --index-filter 'git rm --cached --ignore-unmatch *.weights' --prune-empty --tag-name-filter cat -- --all
sudo git filter-branch --force --index-filter 'git rm --cached --ignore-unmatch *.caffemodel' --prune-empty --tag-name-filter cat -- --all
