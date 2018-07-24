# Kanaloa
This is the official Github for team Kanaloa.  Our main website can be found [here](http://rip.eng.hawaii.edu/research/unmanned-x-systems/).

Do not commit edits to the master branch **until they have been fully vetted in hardware**!

## 1. Directory listing
 - `PrimerDocuments: detailed notes on a topic.
 - `Projects`: files related to a specific project or event.
 - `SurfaceVehicles`: files related to the operation of a specific surfave vehicle.
 - `Tutorials`: detailed notes on the operation, installation, or maintinence of a specific software of hardware.  
 
 The difference between `PrimerDocuments` and `Tutorials` may be difficult to discern at first.  Tutorials should answer the question of "how do I operate this?", primer documents should answer the question of "how does this work?".

## 2. Directory and filename convention

### 2.1. Directory naming convention
All directories shall start with a capital letter.  If the directory name has multiple words, capitalize the first letter of subsequent words; you may use underscores `_` when necessary, but do so sparingly.  No spaces in directory names.  Example: `SampleDirectory`.

### 2.2. File naming convention
There are two types of files: those that require manual version control, and those that do not.  Github autmatically handles version control; however, there is still value in manually archiving dated versions of files when major milestons are reached (particularlly when working in the field with limited internet access).  Examples of files that require manual version control include: code, notes, work in progress, etc.  You will need to use your judgement when deciding which files need manual version control.

 - **For files requiring manual version control**, the filenames shall start with the date that file was originally created.  This is in the format `YYYYMMDD_name`.  `name` must start with a lowercase letter.  If `name` has multiple words, capitalize the first letter of subsequent words; you may use underscores `_` when necessary, but try to avoid doing this.  No spaces in filenames.  Example: `20180101_sampleFilename.extension`.
 - **For files that do not require manual version control**, follow the same naming convention as above, omitting the date.  Example: `sampleFilename.extension`.

**Exceptions**:
 - Matlab (in Linux) dislikes filenames that start with numbers.  For this reason, Matlab script `.m` filenames should begin with the letter `m`.  Other than this, the naming convention remains the same.  Example: `m20180101_sampleMatlabFilename.m`
 - Arduino IDE (in Linux) dislikes filenames that start with numbers.  It also prefers that .ino scrips sit inside a directory with the same name as the filename.  For this reason, Arduino script `.ino` filenames should begin with the letter a.  The script should sit inside a directory named identical to the filename.  Other than this, the naming convention remains the same.  Example: `.../a20180101_sampleArduinoFilename/a20180101_sampleArduinoFilename.ino`

## 3. Writing readme.md files
For writing tutorials and/or primer dicuments in Git, put the document in its own directory.  Name the directory a properly descriptive name (e.g. "InstallationInstructions", "PackageLists", "WritingAPackage", etc.), then name the document inside the directory "readme.md".  This will allow you to leverage Git's [built-in writing and formatting syntax](https://help.github.com/articles/basic-writing-and-formatting-syntax/), which will allow you to write nicely-formatted documents like the one you are reading right now!  
