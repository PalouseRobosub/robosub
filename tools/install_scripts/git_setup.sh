#!/bin/bash
#execute this script to setup the gitconfig for the local repo
#change the LOCALITY variable to 
#	'--global' for all repos of current user
#   '--system' for all users of the current machine

LOCALITY='--local'

#set color terminal mode
git config $LOCALITY  color.ui auto
#only push explictly defined branch
git config $LOCALITY push.default upstream
#filter for converted spaces to tabs
git config $LOCALITY filter.tab.clean 'unexpand --tabs=4 --first-only'

