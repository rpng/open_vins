#!/bin/bash

# sudo apt install clang-format
find ov_core/ -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;
find ov_eval/ -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;
find ov_init/ -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;
find ov_msckf/ -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -exec clang-format -style=file -i {} \;

# sudo apt install libc++-dev clang-tidy
#find . -regex '.*\.\(cpp\|hpp\|cu\|c\|h\)' -exec clang-tidy {} \;

