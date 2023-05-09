#!/bin/bash

# sudo apt install cloc

printf '%*s\n' "${COLUMNS:-$(tput cols)}" '' | tr ' ' -
echo -e " OV_CORE SOURCE FILES"
printf '%*s\n' "${COLUMNS:-$(tput cols)}" '' | tr ' ' -
cloc --by-file ov_core/src/* #| grep "ov_core/src/"


printf '\n%*s\n' "${COLUMNS:-$(tput cols)}" '' | tr ' ' -
echo -e " OV_INIT SOURCE FILES"
printf '%*s\n' "${COLUMNS:-$(tput cols)}" '' | tr ' ' -
cloc --by-file ov_init/src/* #| grep "ov_init/src/"


printf '\n%*s\n' "${COLUMNS:-$(tput cols)}" '' | tr ' ' -
echo -e " OV_MSCKF SOURCE FILES"
printf '%*s\n' "${COLUMNS:-$(tput cols)}" '' | tr ' ' -
cloc --by-file ov_msckf/src/* #| grep "ov_msckf/src/"



