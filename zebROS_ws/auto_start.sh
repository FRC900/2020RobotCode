# Would be a part of the .bashrc file and run upon entering your container.

cd ~/2019Offseason/zebROS_ws/

read -p "Pull code? (rebase) Y/n" -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
	git pull -r
fi

read -p "Run native build? Y/n" -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
	./native_build.sh
fi

read -p "Source ROSStandard? Y/n" -n 1 -r
echo    # (optional) move to a new line
if [[ $REPLY =~ ^[Yy]$ ]]
then
	source ~/2019Offseason/zebROS_ws/ROSStandard.sh
	echo 'Sourced ROSStandard.sh'
fi
