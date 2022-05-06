#!/bin/bash
# SPDX-License-Identifier: GPL-2.0+
# Copyright © 2016,2020 IBM Corporation
#
# This script checks the unrelocated code of a vmlinux for "suspicious"
# branches to relocated code (head_64.S code).

# Have Kbuild supply the path to objdump and nm so we handle cross compilation.
objdump="$1"
vmlinux="$2"

#__end_interrupts should be located within the first 64K

end_intr=0x$(
$objdump -R "$vmlinux" -d --start-address=0xc000000000000000           \
		 --stop-address=0xc000000000010000 |
grep '\<__end_interrupts>:' |
awk '{print $1}'
)

BRANCHES=$(
$objdump -R "$vmlinux" -D --start-address=0xc000000000000000           \
		--stop-address=${end_intr} |
grep -e "^c[0-9a-f]*:[[:space:]]*\([0-9a-f][0-9a-f][[:space:]]\)\{4\}[[:space:]]*b" |
grep -v '\<__start_initialization_multiplatform>' |
grep -v -e 'b.\?.\?ctr' |
grep -v -e 'b.\?.\?lr' |
sed 's/://' |
awk '{ print $1 ":" $6 ":0x" $7 ":" $8 " "}'
)

for tuple in $BRANCHES
do
	from=`echo $tuple | cut -d':' -f1`
	branch=`echo $tuple | cut -d':' -f2`
	to=`echo $tuple | cut -d':' -f3 | sed 's/cr[0-7],//'`
	sym=`echo $tuple | cut -d':' -f4`

	if (( $to > $end_intr ))
	then
		if [ -z "$bad_branches" ]; then
			echo "WARNING: Unrelocated relative branches"
			bad_branches="yes"
		fi
		printf '%s %s-> %s %s\n' "$from" "$branch" "$to" "$sym"
	fi
done

$all_good

}
