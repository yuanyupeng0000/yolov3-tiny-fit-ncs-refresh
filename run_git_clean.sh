sudo git filter-branch --force --index-filter 'git rm --cached --ignore-unmatch *.264' --prune-empty --tag-name-filter cat -- --all
sudo git push -f
