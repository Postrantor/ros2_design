#!/bin/bash

if [[ $# -lt 2 ]] ; then
    echo 'Usage: ./text_replace.sh DIRECTORY EXTENSION'
    exit 0
fi

dir_path=$1

if [[ $# -eq 2 ]]; then
  extension=$2
else
  extension=".md" #设定默认后缀名
fi

operations=(
    's///g'
    's/(/\(/g'
    's/)/\)/g'
    's/。/。/g'
    's/，/，/g'
    # 's/"/`/g'
    's/；/；/g'
    's/。/。/g'
    's/，/，/g'
    's/\]/]/g'
    's/\\[/[/g'
    's/\\ \[/[/g'
    's/_/_/g'
    's/\_/_/g'
    's///g'
    's/镜像/镜像/g'
    's/recipes/recipes/g'
    's///g'
)

function traverse_dir() {
    for file in $1/*
    do
        if [[ -d $file ]]; then
            traverse_dir $file
        else
            extension=$
            # if [[ $extension == $2 ]]; then
                echo "Processing $file"
                for i in "$"
                do
                    sed -i "$i" "$file"
                done
            # fi
        fi
    done
}

traverse_dir $dir_path $2

echo "Replacement Completed."
