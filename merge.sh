# rem
git fetch origin master
git checkout origin/master -f
git branch base-$(date +"%Y%m%d")
git merge sm7125 --no-edit
git merge clk --no-edit
git merge power --no-edit
git merge gpu --no-edit
git merge panel --no-edit
git merge nt36xxx --no-edit
git merge wled --no-edit
git merge camss --no-edit
git merge cmos --no-edit
git merge dm-user --no-edit
git merge misc --no-edit
git merge sound --no-edit
git merge pd-mapper --no-edit
git rebase origin/master
git branch working-$(date +"%Y%m%d")
git push 99degree HEAD:refs/heads/working-20240530
