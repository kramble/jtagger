# NB wcmd is ~/bin/wcmd ... vis ...
#    wine /tmp/aq/10.1/quartus/bin/$1 $2 $3 $4 $5 $6 $7 $8 $9 ${10} ${11} ${12} ${13} ${14} ${15}
cd output_files
# wcmd quartus_cpf.exe -c -q 24.0MHz -g 3.3 -n p system.sof system.svf
wcmd quartus_cpf.exe -c --option=bitstream_compression=off mp3.sof mp3.rbf
tput init	# fix colours (yes quartus, very pretty but rude, why?)
cp mp3.rbf ../mp3.rbf
cp mp3.rbf ../system.rbf
