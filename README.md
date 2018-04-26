# kanaloa
This is the official Github for team Kanaloa.

Do not commit edits to the master branch UNTIL THEY HAVE BEEN FULLY VETTED IN HARDWARE!  

# directory and filename convention

Directory naming convention:
All directories must start with a capital letter.  If the directory name has multiple words, capitalize the first letter of subsequent words.  No spaces in directory names.  Example: "SampleDirectory"

File naming convention:
All filenames must start with the date that file was originally created.  This is in the format YYYY.MM.DD_name.  "name" must start with a lowercase letter.  If "name" has multiple words, capitalize the first letter of subsequent words.  No spaces in filenames.  Example: "2018.01.01_sampleFilename.extension"

Exceptions:
Matlab (in Linux) dislikes periods (".") in filenames.  It also dislikes filenames that start with numbers.  For this reason, Matlab script (.m) filenames should begin with the letter m, and forego the "." in the date.  Other than this, the naming convention remains the same.  Example: "m20180101_sampleMatlabFilename.m"

Arduino IDE (in Linux) dislikes periods (".") in filenames.  It also dislikes filenames that start with numbers.  It also prefers to sit inside a directory with the same name as the filename.  For this reason, Arduino script (.ino) filenames should begin with the letter a, and forego the "." in the date.  The script should sit inside a directory named identical to the filename.  Other than this, the naming convention remains the same.  Example: "a20180101_sampleArduinoFilename.ino"
