# Kanaloa
This is the official Github for team Kanaloa.  Our main website can be found [here](http://rip.eng.hawaii.edu/research/unmanned-x-systems/).

Do not commit edits to the master branch **until they have been fully vetted in hardware**!

## 1. Directory listing
 - `Documentation`: tutorials, primer documents, and detailed notes on specific topics.
 - `Projects`: files related to a specific project or event.
 - `SurfaceVehicles`: files related to the operation of a specific surface vehicle.

## 2. Directory and filename convention

### 2.1. Directory naming convention
All directories shall start with a capital letter.  Utilize [camel case](https://en.wikipedia.org/wiki/Camel_case) for multiple words (capitalize the first letter of subsequent words with no spaces; you may use underscores `_`, but do so sparingly).  Example: `SampleDirectory`.

### 2.2. File naming convention

 - **For files requiring manual version control**, the filenames shall start with the date that file was originally created.  This is in the format `YYYYMMDD_name`.  `name` must start with a lowercase letter.  Utilize [camel case](https://en.wikipedia.org/wiki/Camel_case) for multiple words capitalize the first letter of subsequent words with no spaces; you may use underscores `_`, but do so sparingly).  Example: `20180101_sampleFilename.extension`.  Examples of files that require manual version control include: code reflecting major hardware revisions, notes, work in progress, etc.  You will need to use your judgement when deciding which files need manual version control.  Github automatically handles version control by nature; however, it makes sense for manual version control in some cases.  
 - **For files that do not require manual version control**, follow the same naming convention as above, omitting the date and underscore.  Example: `sampleFilename.extension`.

**Exceptions**:
 - Matlab (in Linux) dislikes filenames that start with numbers.  For this reason, Matlab script `.m` filenames should begin with the letter `m`.  Other than this, the naming convention remains the same.  Example: `m20180101_sampleMatlabFilename.m`
 - Arduino IDE (in Linux) dislikes filenames that start with numbers.  It also prefers that .ino scrips sit inside a directory with the same name as the filename.  For this reason, Arduino script `.ino` filenames should begin with the letter a.  The script should sit inside a directory named identical to the filename.  Other than this, the naming convention remains the same.  Example: `.../a20180101_sampleArduinoFilename/a20180101_sampleArduinoFilename.ino`

## 3. Writing readme.md files
For writing tutorials and/or primer dicuments in Git, put the document in its own directory.  Name the directory a properly descriptive name (e.g. "InstallationInstructions", "PackageLists", "WritingAPackage", etc.), then name the document inside the directory "readme.md".  This will allow you to leverage Git's [built-in writing and formatting syntax](https://help.github.com/articles/basic-writing-and-formatting-syntax/), which will allow you to write nicely-formatted documents like the one you are reading right now!  
