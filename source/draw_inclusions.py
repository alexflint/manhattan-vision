#!/usr/bin/python

import sys;
import os;
import fnmatch;
import re;

def main():
    paths = ["."] if (len(sys.argv) <= 1) else sys.argv[1:]
    for path in paths:
        files = os.listdir(path)
        srcs = fnmatch.filter(files, "*.h") + fnmatch.filter(files, "*.cpp") + fnmatch.filter(files, "*.tpp")

        pat = re.compile('^#include "([^"]*)"');

        filenames = [];
        inclusions = [];

        for file in srcs:
            filename = os.path.basename(file);
            filenames += [filename];
            #print file
            if filename[-2:] == ".h":
                with open(file, "r") as f:
                    for line in f:
                        match = re.match(pat, line)
                        if match is not None:
                            inclusions += ((filename, match.group(1)),)

    # pick the unique ones
    filenames = set(filenames)
    inclusions = set(inclusions)

    # write
    print '''
digraph headers {
size="50,50";
node [color=lightblue2, style=filled];
''';
    #print '\tgraph[page="8.5,11",size="7.5,7",ratio=fill,center=1];\n')
    for link in inclusions:
        print '\t"%s" -> "%s";' % link
    print '}'

main();
