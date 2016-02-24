#!/bin/bash
#--------------------------------------------------------------------------
# [Goncalo Andre] compile.sh
#                 automatic generation of latex output documents
# 02-2016
#--------------------------------------------------------------------------

#Error handling: set a trap to catch and decipher script command errors
#reporting function: print line number of command, code and error message
error_report() {
  local parent_lineno="$1"
  local message="$2"
  local code="${3:-1}"
  if [[ -n "$message" ]] ; then
    echo "Error on or near line ${parent_lineno}: ${message}; exiting with status ${code}"
  else
    echo "Error on or near line ${parent_lineno}; exiting with status ${code}"
  fi
  exit "${code}"
}
#trap to catch errors
trap 'error_report ${LINENO}' ERR


#Main script
#--------------------------------------------------------------------------
#help message
usage="Usage: compile.sh [-h|--help] [-c|--clean] [-d|--dvi] [-r|--redirect]\n\t--help\t\
Display this message and exit.\n\t--clean\tClean temporary files and exit.\
\n\t--dvi\tOuput .dvi file instead of .pdf.\n\t--redirect\tRedirect (append) output to file\n\t"
description="Automatically generate the final document from current latex \
sources"

#--------------------------------------------------------------------------
#default configuration
input=manual #input filename prefix
output=manual #output filename prefix
format=pdf  #extension of the output filename to generate
outputdir=tmp #directory where auxiliary files are saved to
redirect=0 #write to stdout (by default there is no redirection)
logfile=$input.complog #Cannot use .log extension because that file is already used by LaTeX

#--------------------------------------------------------------------------
#functions
#Clean funtion: remove unimportant latex output files
clean ()
{
    rm -rf ./$outputdir
    rm -f ./$output.$format
    rm -f ./$logfile
    echo "temporary files cleaned"
}

#Lat function: run latex
lat ()
{
    #set latex information to work on directory tree
    #http://tex.stackexchange.com/questions/31925/install-package-in-subfolder
    #avoid warnings because of packages not in root directory
    export TEXINPUTS=./configuracao//:${TEXINPUTS}

    #run latex
    if ! pdflatex -file-line-error -halt-on-error -output-directory=$outputdir -output-format=$format $input; then #run latex
    #http://blog.sanctum.geek.nz/testing-exit-values-bash/
        echo "An error has occurred while running latex. Return code $$."
        exit;
    fi
}

#bibt function: run bibtex. cannot use bibtex, as bash will confuse the function name with the actual bibtex command
bib ()
{
    #http://tex.stackexchange.com/questions/12686/how-do-i-run-bibtex-after-using-the-output-directory-flag-with-pdflatex-when-f
    #for this to work, I had to:
    #sudo vim /usr/share/texlive/texmf-dist/web2c/texmf.cnf
    #find the openout_any = p line and change it either to a or r
    #and set correctly the TEXMFOUTPUT, as bellow
    export TEXMFOUTPUT="$outputdir"

    if ! bibtex $outputdir/$input; then #run bibtex
    #http://blog.sanctum.geek.nz/testing-exit-values-bash/
        echo "An error has occurred while running bibtex Return code $$."
        exit;
    fi
}

#--------------------------------------------------------------------------
#parse invocation options
while [[ $# > 0 ]]
do
key="$1"

case $key in
    -h|--help)
    #print help and exit
    echo -e "$usage"
    echo -e "$description"
    exit
    shift # past argument
    ;;
    -c|--clean)
    #clean temporary files from previous jobs and exit
    clean
    exit
    shift # past argument
    ;;
    -d|--dvi)
    #Configure the output format as dvi when calling latex
    format=dvi
    shift # past argument
    ;;
    -r|--redirect)
    #redirect output to file
    redirect=1
    echo "redirect option does nothing. Exiting"
    exit
    #echo "redirecting output to" $logfile
    #echo "New job started at $(date)" >>$logfile
    shift # past argument
    ;;
    *)
    echo "Unknow  argument. Use --help see program usage"
    exit
    shift # past argument
    ;;
esac
done


#main job
#clean and compile
clean
#Create directory if it doesn't exist
mkdir -p $outputdir
lat
#for now bibtex is not used
bib
lat
lat
#Rename output file
mv ./$outputdir/$input.$format "$output".$format

echo "Job done on file" "$output".$format
