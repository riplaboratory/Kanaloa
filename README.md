# Kanaloa
This is the official Github for team Kanaloa.  Our main website can be found [here](http://rip.eng.hawaii.edu/research/unmanned-x-systems/).

Do not commit edits to the master branch **until they have been fully vetted in hardware**!

## Directory and filename convention

### 1. Directories:
All directories shall start with a capital letter.  If the directory name has multiple words, capitalize the first letter of subsequent words; you may use underscores `_` when necessary, but try to avoid doing this.  No spaces in directory names.  Example: `SampleDirectory`.

### 2. Files:
There are two types of files: those that require manual version control, and those that do not.  Examples of files that require manual version control include: code, notes, work in progress, etc.  You will need to use your judgement when deciding which files need manual version control.  For these files, the filenames shall start with the date that file was originally created.  This is in the format `YYYY.MM.DD_name`.  `name` must start with a lowercase letter.  If `name` has multiple words, capitalize the first letter of subsequent words; you may use underscores `_` when necessary, but try to avoid doing this.  No spaces in filenames.  Example: `2018.01.01_sampleFilename.extension`.

Files that do not require manual version control shall follow the same convention, omitting the date.  Example: `sampleFilename.extension`.

### 3. Exceptions:
Matlab (in Linux) dislikes periods `.` in filenames.  It also dislikes filenames that start with numbers.  For this reason, Matlab script `.m` filenames should begin with the letter `m`, and forego the `.` in the date.  Other than this, the naming convention remains the same.  Example: `m20180101_sampleMatlabFilename.m`

Arduino IDE (in Linux) dislikes periods `.` in filenames.  It also dislikes filenames that start with numbers.  It also prefers to sit inside a directory with the same name as the filename.  For this reason, Arduino script `.ino` filenames should begin with the letter a, and forego the `.` in the date.  The script should sit inside a directory named identical to the filename.  Other than this, the naming convention remains the same.  Example: `a20180101_sampleArduinoFilename.ino`

## Writing readme.md files
For writing tutorials and/or primer dicuments in Git, put the document in its own directory.  Name the directory a properly descriptive name (e.g. "InstallationInstructions", "PackageLists", "WritingAPackage", etc.), then name the document inside the directory "readme.md".  This will allow you to leverage Git's [built-in writing and formatting syntax](https://help.github.com/articles/basic-writing-and-formatting-syntax/), which will allow you to write nicely-formatted documents like the one you are reading right now!  
