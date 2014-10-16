#!/usr/bin/env perl

my $file = $ARGV[0];
my $dir = $ARGV[1];

open (MYFILE, $file); 
while (<MYFILE>) { 
	chomp; 
	my $line = $_;
	#print "$line\n";
	if($line =~ /(\w+)\s(\w+\.cf)\s(\w+\.mf)/){
		my $cf = $dir.$2;
		my $mf = $dir.$3;
		#print "$cf\n";
		#print "$mf\n";
		system("java","simulator.framework.Elevator","-cf",$cf,"-mf",$mf);

	}

	} 

close (MYFILE);