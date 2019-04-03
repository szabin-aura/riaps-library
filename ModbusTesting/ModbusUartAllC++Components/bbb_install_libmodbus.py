#!/usr/bin/python3
'''
Script for installing libmodbus on BBBs

Created on April 2, 2019

Arguments:
    optional argument:
    - ``-H | --hosts hostnames``:  list of hostnames (comma separated), VM will be rekeyed

If specific hostnames are not given, the command will be called for all hosts
listed in /usr/local/riaps/etc/riaps_hosts.conf

@author: marymetelko
'''

import os
import sys
import shlex
import argparse
import subprocess

def bash(cmd):
    t = shlex.split(cmd)
    print("=== "+str(t))
    subprocess.run(t)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-H", "--hosts", default="", help="list of hostnames, comma separated, for a complete reconfiguration")    # List of hostnames to used instead of system configured file
    args = parser.parse_args()

    fcmd = "fab"
    fflag = "-f"
    fpath = "/usr/local/lib/python3.6/dist-packages/riaps/fabfile/"
    fhost = "-H"

    if os.path.isdir(fpath):
        if args.hosts:
            print("Installing libmodbus on hostnames: ",args.hosts)
            s = str.join(' ',(fcmd, fflag, fpath, fhost, args.hosts))
        else:
            print("Installing libmodbus on hostname(s) from /usr/local/riaps/etc/riaps_hosts.conf")
            s = str.join(' ',(fcmd, fflag, fpath))

        finstruct = 'sys.run:"git clone https://github.com/cmjones01/libmodbus.git"'
        icmd = str.join(' ',(s, finstruct))
        bash(icmd)

        finstruct = 'sys.run:"sudo apt-get install autoconf libtool pkg-config -y"'
        icmd = str.join(' ',(s, finstruct))
        bash(icmd)

        finstruct = 'sys.run:"cd libmodbus; ./autogen.sh"'
        icmd = str.join(' ',(s, finstruct))
        bash(icmd)

        finstruct = 'sys.run:"cd libmodbus; ./configure"'
        icmd = str.join(' ',(s, finstruct))
        bash(icmd)

        finstruct = 'sys.run:"cd libmodbus; make"'
        icmd = str.join(' ',(s, finstruct))
        bash(icmd)

        finstruct = 'sys.sudo:"cd libmodbus; make install"'
        icmd = str.join(' ',(s, finstruct))
        bash(icmd)

    else:
        print('RIAPS Fabfile is not installed, please update the riaps-pycom installation.')

    print("Installation of libmodbus complete on beaglebones.")
