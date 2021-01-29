# Kanaloa
This is the official Github for team Kanaloa.  Our main website can be found [here](http://rip.eng.hawaii.edu/research/unmanned-x-systems/).

Do not commit edits to the Main branch **until they have been fully vetted in hardware**!

## 1. Directory Listing
 - `Documentation`: tutorials, primer documents, and detailed notes on specific topics.
 - `Projects`: files related to a specific project or event.
 - `SurfaceVehicles`: files related to the operation of a specific surface vehicle.

## 2. Directory and Filename Convention

### 2.1. Directory Naming Convention
All directories shall start with a capital letter.  Utilize [camel case](https://en.wikipedia.org/wiki/Camel_case) for multiple words (capitalize the first letter of subsequent words with no spaces; you may use underscores `_`, but do so sparingly).  Example: `SampleDirectory`.

### 2.2. File Naming Convention

 - **For files requiring manual version control**, the filenames shall start with the date that file was originally created.  This is in the format `YYYYMMDD_name`.  `name` must start with a lowercase letter.  Utilize [camel case](https://en.wikipedia.org/wiki/Camel_case) for multiple words capitalize the first letter of subsequent words with no spaces; you may use underscores `_`, but do so sparingly).  Example: `20180101_sampleFilename.extension`.  Examples of files that require manual version control include: code reflecting major hardware revisions, notes, work in progress, etc.  You will need to use your judgement when deciding which files need manual version control.  Github automatically handles version control by nature; however, it makes sense for manual version control in some cases.  
 - **For files that do not require manual version control**, follow the same naming convention as above, omitting the date and underscore.  Example: `sampleFilename.extension`.

**Exceptions**:
 - Matlab (in Linux) dislikes filenames that start with numbers.  For this reason, Matlab script `.m` filenames should begin with the letter `m`.  Other than this, the naming convention remains the same.  Example: `m20180101_sampleMatlabFilename.m`
 - Arduino IDE (in Linux) dislikes filenames that start with numbers.  It also prefers that .ino scrips sit inside a directory with the same name as the filename.  For this reason, Arduino script `.ino` filenames should begin with the letter a.  The script should sit inside a directory named identical to the filename.  Other than this, the naming convention remains the same.  Example: `.../a20180101_sampleArduinoFilename/a20180101_sampleArduinoFilename.ino`

## 3. Writing README.md Files
For writing tutorials and/or primer dicuments in Git, put the document in its own directory.  Name the directory a properly descriptive name (e.g. "InstallationInstructions", "PackageLists", "WritingAPackage", etc.), then name the document inside the directory "README.md".  This will allow you to leverage Git's [built-in writing and formatting syntax](https://help.github.com/articles/basic-writing-and-formatting-syntax/), which will allow you to write nicely-formatted documents like the one you are reading right now!  

## 4. Issue Usage Convention
Issues are used to help keep track of tasks, enhancements, and bugs for the project. Before starting work on code you will want to create an issue and when the code is completed and working, you can close the issue so other members know the task has been completed. Issues should be detailed and focused goals that you want to complete in a week. 

### 4.1. Issue Naming Convention
Issues should be a brief description of the goal. For example, an issue named "Issues SOP" can have the following description:
```
### Create a guide on how to create an issue for the purpose of this repository
- [x] Naming convention
- [ ] Using assignees
- [ ] Using labels
- [ ] Adding to project boards
```
Issues support styling with Markdown.

### 4.2. Creating an Issue

### 4.3. Closing an Issue

## 5. Branch Usage Convention
Branches are used to create an instance of the Main branch. This feature allows you to make enhancements, fix bugs, or try out new ideas without messing up the existing working files. Whenever working on code, work should be done on a separate branch to help avoid merge conflicts or buggy code from being in main branch. The main branch should contain only the most up-to-date working files.

### 5.1. Branch Naming Convention
Your branch name should be short but also descriptive. For example, when updating this Standard Operating Procedure (SOP), the branch name is `update-github-sop`. The only work being done in this branch is work related to this SOP (`README.md`).

### 5.2. Creating a Branch

### 5.3. Merging Into Main

## 6. Project Board Usage
Project boards allows work to be organize and prioritized. It can be used to create a checklist or a roadmap to work towards completing a goal. It allows an easy view of task that still need to be done, task in progress, and task that have been done. This feature can be automated so that when a pull request goes through, your task is marked as done and the issue is closed. This allows for a visual to track the progress of a project.

### 6.1. Project Board Naming Convention
Your project board name should be a short description of the overall goal you or your team is trying to achieve. Your description should be the general goal of the project. The issues you create for this project will be the goal broken down into more focused task that work towards the completion of it.

### 6.2. Creating a Project Board

## Extra Resources
- [GitHub Desktop](https://desktop.github.com/) (this is a great UI for those starting to learn GitHub)
- [GitHub Documentation](https://docs.github.com/en)
- [GitHub CLI](https://cli.github.com/manual/) (for more advanced users and/or cases)