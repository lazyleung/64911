#!/bin/bash

# This template generates passenger files for acceptance testing the elevator.
#
# Passengers all enter on floor one in a randomized hall, and exit on floor 2.
# You should update this so that passengers can begin and end in any valid 
# hallway. If you want to add additional parameters (e.g. timing, uppeak 
# bias, etc.) you may do so. Be sure that your usage accurately describes all
# parameters used. 
#
# .Pass files should start with a comment describing the test, including what 
# arguments were used to generate it. 
# Subsequent lines are passenger injections and have the following format:
#
#           Time	Start Floor	Start Hallway	End Floor	End Hallway 

MIN_ARG_NUM=2;
usage=$'--- Usage ---
Required: 
First Argument - Number of passengers to be injected
Second Argument - Base name of Output file
Optional:
-t maximum time interval between passenger injections. each interval is randomly generated in the range [1 t]. 
   Default value is 5
-n number of files to be generated. The output file will have the format [name_base]0.pass, [name_base]1.pass, ...  
   Default value is 1
-h print usage message\n';
example='--- Example ---
./GeneratorTemplate.sh 20 test -n 10 -t 5';
if [ $# -lt $MIN_ARG_NUM ] 
    then 
        #Replace these with actual usage directions
        echo "$usage";
        echo "$example";
else
        PASS_NUM=$1; #Read the arguments into local variables
        OUTFILE=$2;
        t=5; #default time interval
        n=1; #default number of files to be generated
        while [ "$1" != "" ];
        do
            case $1 in
                -t ) shift; t=$1
                    ;;
                -n ) shift; n=$1
                    ;;
                -h ) echo "$usage";
                     exit;
            esac
            shift
        done

        n_c=0; #number of files generated

        while (("$n_c" < "$n" ));
        do
            echo ";Random Generated Pass file. Number of Passengers = $PASS_NUM" > $OUTFILE;
        t_s=0;
        for ((i = 1; i <= $PASS_NUM; i++));
            do
                hs_rand=$(($RANDOM % 2 )); #starting hall random seed 
                hd_rand=$(($RANDOM % 2 )); #destination hall random seed
                t_s=$t_s+$((($RANDOM % t)+1));

                f_s=$((($RANDOM % 8)+1)); #starting floor
                f_d=$((($RANDOM % 8)+1)); #destination floor

                while [ $f_s -eq $f_d ] 
                do
                    f_d=$((($RANDOM % 8)+1)); #generate new destination floor
                done
                s="$(($t_s))s "; #Passenger insertion time

                s="$s $f_s "; #Passenger starts on floor f_s ...
                if [ "$f_s" -eq "2" ]
                    then
                    s="$s BACK ";
                else
                    if [ "$f_s" -eq "7" -o "$f_s" -eq "1" ]
                        then
                        case $hs_rand in #... in a random hallway
                        0 )
                            s="$s FRONT ";
                            ;;
                        1 )
                            s="$s BACK  ";
                            ;;
                        esac
                    else
                        s="$s FRONT ";
                    fi
                fi
                
                s="$s $f_d "; #Passenger ends on floor f_d

                if [ "$f_d" -eq "2" ]
                    then
                    s="$s BACK ";
                else
                    if [ "$f_d" -eq "7" -o "$f_d" -eq "1" ]
                        then
                        case $hs_rand in #... in a random hallway
                        0 )
                            s="$s FRONT ";
                            ;;
                        1 )
                            s="$s BACK  ";
                            ;;
                        esac
                    else
                        s="$s FRONT ";
                    fi
                fi
                echo "$s" >> "$OUTFILE$n_c.pass"
            done
            n_c=$((n_c+1));
        done        
                
fi

