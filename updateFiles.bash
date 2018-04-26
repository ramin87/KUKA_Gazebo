echo "===================================================="
echo "===================================================="
git fetch --all
git status
echo "===================================================="
git diff origin/master --stat
echo "===================================================="

read -p "Continue updating files? [y/n]" -n 1 -r
echo    # (optional) move to a new line
echo "===================================================="
if [[ $REPLY =~ ^[Yy]$ ]]
then
    git add -A
    DATE=`date +%Y-%m-%d:%H:%M:%S`
    git commit -m $DATE
    # git reset --hard origin/master
    git pull --no-edit
    # git merge
    echo "===================================================="
    git status
    echo "===================================================="
    # git config --global push.default simple
    git push
    echo "===================================================="
    git status
    echo "----------------------------------------------------"
    echo "----------------------------------------------------"    
fi
    
