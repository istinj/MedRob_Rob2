#!/bin/bash
cd /home/istin/Documenti/3_MR2_Proj/SOFA_BUILD/sofa/v15.12/build/bin
clear
read -n1 -r -p "Press space to view scena_04..." key
if [ "$key" = '' ]; then
	./runSofa /home/istin/Documenti/3_MR2_Proj/Software/scena_04.scn 
fi

read -n1 -r -p "Press space to view scena_06" key
clear
if [ "$key" = '' ]; then
	./runSofa /home/istin/Documenti/3_MR2_Proj/Software/scena_06.scn 
fi

read -n1 -r -p "Press space to view scena_05" key
clear
if [ "$key" = '' ]; then
	./runSofa /home/istin/Documenti/3_MR2_Proj/Software/scena_05.scn 
fi

read -n1 -r -p "Press space to view dentristy_python" key
clear
if [ "$key" = '' ]; then
	./runSofa /home/istin/Documenti/3_MR2_Proj/SOFA_BUILD/sofa/v15.12/src/examples/Tutorials/StepByStep/Dentistry_Python/7_CompleteScene.scn 
fi

read -n1 -r -p "Press space to view the fake plugin" key
clear
if [ "$key" = '' ]; then
	cd /home/istin/Documenti/3_MR2_Proj/SOFA_BUILD/sofa/v15.12/build
	cmake-gui
	echo "wait for building"
	sleep 3
	make -j4
	echo "open the issue's folder"
	sleep .7
	nautilus /home/istin/Documenti/3_MR2_Proj/SOFA_BUILD/sofa/v15.12/src/modules/SofaComponentMain
fi

#./runSofa /home/istin/Documenti/3_MR2_Proj/Software/scena_04.scn
#./runSofa /home/istin/Documenti/3_MR2_Proj/Software/scena_06.scn
#./runSofa /home/istin/Documenti/3_MR2_Proj/Software/scena_05.scn
#./runSofa /home/istin/Documenti/3_MR2_Proj/SOFA_BUILD/sofa/v15.12/src/examples/Tutorials/StepByStep/Dentistry_Python/7_CompleteScene.scn



