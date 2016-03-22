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
usage="Usage: compile.sh [-h|--help] [-c|--clean] [-d|--dvi] [-r|--redirect] [-o|--once]\n\t--help\t\t\
Display this message and exit.\n\t--clean\t\tClean temporary files and exit.\
\n\t--dvi\t\tOuput .dvi file instead of .pdf.\n\t--redirect\tRedirect (append) output to file\n\t--once\t\tRun latex once and exit\n\t"
description="Automatically generate the final document from current latex \
sources"

#--------------------------------------------------------------------------
#default configuration
input=dissertacao #input filename prefix
output=dissertacao #output filename prefix
format=pdf  #extension of the output filename to generate
outputdir=tmp #directory where auxiliary files are saved to
redirect=0 #write to stdout (by default there is no redirection)
initiallogfile=initialruns.log
finallogfile=finalrun.log

#--------------------------------------------------------------------------
#functions
#Clean funtion: remove unimportant latex output files
clean ()
{
    rm -rf ./$outputdir
    rm -f ./$output.$format
    rm -f ./$initiallogfile
    rm -f ./$finallogfile
    echo "temporary files cleaned"
    #re-create directory
    mkdir -p $outputdir
}

#highlight_err_info: highlight latex compilation errors and warning on log file
highlight_err_info ()
{
echo "Errors and warnings found on" $1 ":"
echo "For info on how to detect warning origin, see: http://tex.stackexchange.com/questions/235051/an-equivalent-of-file-line-error-for-warnings"
cat $1 | grep -Ein --color "error|warning" || true
}

#Lat function: run latex
lat ()
{
    #set latex information to work on directory tree
    #http://tex.stackexchange.com/questions/31925/install-package-in-subfolder
    #avoid warnings because of packages not in root directory
    export TEXINPUTS=./configuracao//:${TEXINPUTS}

    #run latex
    if ! pdflatex --shell-escape -file-line-error -halt-on-error -output-directory=$outputdir -output-format=$format $input; then #run latex
    #http://blog.sanctum.geek.nz/testing-exit-values-bash/
        echo "An error has occurred while running latex. Halting. Return code $$."
        #ouput err info to stderr, because stdout is being redirected to file
        >&2 highlight_err_info $logfile
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
        echo "An error has occurred while running bibtex. Halting. Return code $$."
        #ouput err info to stderr, because stdout is being redirected to file
        >&2 highlight_err_info $logfile
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
    -o|--once)
    #run latex once and exit
    echo "Running latex one time"
    lat
    echo "Output file located at" "./$outputdir/$input.$format"
    exit
    shift # past argument
    ;;
    -r|--redirect)
    #redirect output to file
    redirect=1
    echo "redirect option does nothing. Exiting."
    exit
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

#initial runs
logfile=$initiallogfile
echo "Starting latex. Appending log to" $logfile ".This should produce temporary warnings, that won't show in the last run"
echo "New job started at $(date)" >>$logfile
lat>>$logfile
bib>>$logfile
lat>>$logfile

#final run
logfile=$finallogfile
echo "Starting last latex run. Separating log to" $logfile "to ease error detection"
echo "New job started at $(date)" >>$logfile
lat>$logfile

#Warning highlighting
highlight_err_info $logfile

#copy output file
cp ./$outputdir/$input.$format "$output".$format

echo "Job done on file" "$output".$format

