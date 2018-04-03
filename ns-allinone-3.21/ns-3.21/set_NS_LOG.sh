#!/bin/bash

usage()
{
	echo
	echo "****************** HOW TO USE ********************"
	echo 
	echo "source ./set_NS_LOG.sh [OPTION] [COMPONENT NAME]"
	echo
	echo " OPTION "
	echo
	echo " -h		help "
	echo " D		Disable All Component"
	echo " E		Enable All Component"
	echo " A [COMPONENT NAME] ... [COMPONENT NAME]	Add Additional Log Components"
	echo "								e.g., source ./set_NS_LOG.sh A WifiMac MacLow"
	echo 
	echo " I [COMPONENT NAME] ... [COMPONENT NAME]	Initialize Enabling Components"
	echo "								e.g., source ./set_NS_LOG.sh I WifiMac MacLow"
	echo 
	echo
	echo "**************************************************"
	echo
}

if [ $1 == "-h" ]
then
	usage
elif [ $1 == "D" ]
then
	NS_LOG=""
	echo "All components disabled"
	
# Enable FUNCTION 
elif [ $1 == "E" ]
then
	NS_LOG="*=level_debug|prefix_node|prefix_func|prefix_time"
elif [ $1 == "EE" ]
then
	NS_LOG="*=level_all|prefix_node|prefix_func|prefix_time"
elif [ $1 == "EEE" ]
then
	NS_LOG="*=level_error|prefix_node|prefix_func|prefix_time"

elif [ $1 == "A" ]
then
	for p in $*
	do
		if [ $p == "A" ]
		then
			echo "Add New Compoent"
		else
			echo "$p"
			NS_LOG="$NS_LOG:$p=level_function|prefix_node|prefix_func|prefix_time"
		fi
	done 
elif [ $1 == "AI" ]
then
	for p in $*
	do
		if [ $p == "AI" ]
		then
			echo "Add New INFO Compoent"
		else
			echo "$p"
			NS_LOG="$NS_LOG:$p=level_info|prefix_node|prefix_func|prefix_time"
		fi
	done 

elif [ $1 == "I" ]
then
	for p in $*
	do
		if [ $p == "I" ]
		then
			echo "Initialize NS_LOG Component"
			NS_LOG=""
			FIRST="1"
		elif [ $FIRST == "1" ]
		then
			echo "$p"
			NS_LOG="$p=level_function|prefix_node|prefix_func|prefix_time"
			FIRST="0"
		else 
			echo "$p"
			NS_LOG="$NS_LOG:$p=level_function|prefix_node|prefix_func|prefix_time"
		fi
	done


# Enable DEBUG
elif [ $1 == "EB" ]
then
	NS_LOG="*=level_debug|prefix_node|prefix_func|prefix_time"
elif [ $1 == "AB" ]
then
	for p in $*
	do
		if [ $p == "AB" ]
		then
			echo "Add New DEBUG Compoent"
		else
			echo "$p"
			NS_LOG="$NS_LOG:$p=level_debug|prefix_node|prefix_func|prefix_time"
		fi
	done 
elif [ $1 == "IB" ]
then
	for p in $*
	do
		if [ $p == "IB" ]
		then
			echo "Initialize NS_LOG_DEBUG Component"
			NS_LOG=""
			FIRST="1"
		elif [ $FIRST == "1" ]
		then
			echo "$p"
			NS_LOG="$p=level_debug|prefix_node|prefix_func|prefix_time"
			FIRST="0"
		else 
			echo "$p"
			NS_LOG="$NS_LOG:$p=level_debug|prefix_node|prefix_func|prefix_time"
		fi
	done 




# Enable only ERROR
elif [ $1 == "EE" ]
then
	NS_LOG="*=level_error|prefix_node|prefix_func|prefix_time"
elif [ $1 == "AE" ]
then
	for p in $*
	do
		if [ $p == "AE" ]
		then
			echo "Add New ERROR Compoent"
		else
			echo "$p"
			NS_LOG="$NS_LOG:$p=level_error|prefix_node|prefix_func|prefix_time"
		fi
	done 
elif [ $1 == "IE" ]
then
	for p in $*
	do
		if [ $p == "IE" ]
		then
			echo "Initialize NS_LOG_ERROR Component"
			NS_LOG=""
			FIRST="1"
		elif [ $FIRST == "1" ]
		then
			echo "$p"
			NS_LOG="$p=level_error|prefix_node|prefix_func|prefix_time"
			FIRST="0"
		else 
			echo "$p"
			NS_LOG="$NS_LOG:$p=level_error|prefix_node|prefix_func|prefix_time"
		fi
	done 

elif [ $1 == "IEE" ]
then
	for p in $*
	do
		if [ $p == "IEE" ]
		then
			echo "Initialize NS_LOG_ERROR Component w/o prefix_func"
			NS_LOG=""
			FIRST="1"
		elif [ $FIRST == "1" ]
		then
			echo "$p"
			NS_LOG="$p=level_error|prefix_node|prefix_time"
			FIRST="0"
		else 
			echo "$p"
			NS_LOG="$NS_LOG:$p=level_error|prefix_node|prefix_time"
		fi
	done 

elif [ $1 == "AEE" ]
then
	for p in $*
	do
		if [ $p == "AEE" ]
		then
			echo "Add New ERROR Compoent w/o prefix_func"
		else
			echo "$p"
			NS_LOG="$NS_LOG:$p=level_error|prefix_node|prefix_time"
		fi
	done 


fi

export NS_LOG 
echo
echo $NS_LOG

