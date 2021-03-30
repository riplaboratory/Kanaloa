# Kanaloa
This is the official Github for team Kanaloa.  Our main website can be found [here](http://rip.eng.hawaii.edu/research/unmanned-x-systems/).

Do not commit edits to the Master branch **until they have been fully vetted in hardware**! And approved by Dr. Trimble or the appropriate graduate student mentor.

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
For writing tutorials and/or primer dicuments in Git, put the document in its own directory. Name the directory a properly descriptive name (e.g. "InstallationInstructions", "PackageLists", "WritingAPackage", etc.), then name the document inside the directory `README.md`. This will allow you to leverage Git's [built-in writing and formatting syntax](https://help.github.com/articles/basic-writing-and-formatting-syntax/), which will allow you to write nicely-formatted documents like the one you are reading right now!  

## 4. Branch Usage Convention
Branches are used to create an instance of the Master branch. This feature allows you to make enhancements, fix bugs, or try out new ideas without messing up the existing working files. Whenever working on code, work should be done on a separate branch to help avoid merge conflicts or buggy code from being in master branch. The master branch should contain only the most up-to-date working files. At all times the master branch must be fully operaable and ready to load to a vehicle. We recommend getting familiar with [GitHub Desktop](https://docs.github.com/en/desktop) so that you can see which branch you are working on and will be making commits to.

### 4.1. Branch Naming Convention
Your branch name should be short but also descriptive. For example, when updating this Standard Operating Procedure (SOP), the branch name is `update-github-sop`. The only work being done in this branch is work related to this SOP (`README.md`). 

### 4.2. Creating a Branch
First, make sure you are on the [main page](https://github.com/riplaboratory/Kanaloa) and `master` branch. You create a branch off the currently select branch. If you are not on `master` you could end up merging non-working code down the line. Click where it says `master` and enter your branch name in the text field. Then click on `Create branch: branch-name from 'master'`.

**Alternative Ways**:
- [Creating a branch on GitHub Desktop](https://docs.github.com/en/desktop/contributing-and-collaborating-using-github-desktop/managing-branches)
- [Creating a branch through Git bash](https://www.atlassian.com/git/tutorials/using-branches)

### 4.3. Switching Branches
On GitHub Desktop, make sure that you have [cloned the repository](https://docs.github.com/en/desktop/contributing-and-collaborating-using-github-desktop/cloning-a-repository-from-github-to-github-desktop). Once cloned, you should see `Kanaloa` as the `Current repository` and `Current branch` as `master`. Switch the `Current branch` to the branch that you will be working on.

Note: If you accidentally started work on a different branch, you can switch branches and bring all changes over. 

### 4.4. Merging Into Master
Before merging, make sure that your [branch is up to your latest code](https://docs.github.com/en/desktop/contributing-and-collaborating-using-github-desktop/committing-and-reviewing-changes-to-your-project). Now, on the main page, switch to the branch that you are working on. Click on `Pull request` and you should see:
```
base:master <- compare:branch-name  Able to merge. These branches can be automatically merged.
```
The title will be automatically filled out with your last commit to the branch. Update this to highlight the overall issue that you were handling. On the right-hand side, fill out the fields similar to `Section 4.2`. 

If there is a conflict, you can review the code to see where it is happening. Due to multiple members working on the same repository, there are chance of them happening. Here is the [documentation on how to resolve it](https://docs.github.com/en/github/collaborating-with-issues-and-pull-requests/resolving-a-merge-conflict-on-github).

## 5. Action Items
Action items are currently being tracked using this [Google Sheets](https://docs.google.com/spreadsheets/d/1dgpwZHpQM16Ki9ChiyU2lS0WMApJk8ZLYT5470j_FbE/edit?usp=sharing). 

Please note that this spreadsheet is restricted to those in the Kanaloa Google Drive. If you are a member and do not have access, please contact Dr. Trimble or a graduate student for further assistance.

<!-- THIS SECTION COVERS ISSUES AND PROJECT BOARDS. CURRENTLY (SPRING 2021), IT IS NOT BEING USED - Kevin Nguyen (nk279@hawaii.edu)
## 5. Issue Usage Convention
Issues are used to help keep track of tasks, enhancements, and bugs for the project. Before starting work on code you will want to create an issue and when the code is completed and working, you can close the issue so other members know the task has been completed. Issues should be detailed and focused goals that can be individually completed each week. 

### 5.1. Issue Naming Convention
Issues should be a brief description of the goal. For example, an issue named "Issues SOP" can have the following description:
```
### Create a guide on how to create an issue for the purpose of this repository
- [x] Naming convention
- [ ] Using assignees
- [ ] Using labels
- [ ] Adding to project boards
```
Issues support styling with Markdown.

### 5.2. Creating an Issue
To create an issue, click on the `Issues` tab or [here](https://github.com/riplaboratory/Kanaloa/issues). On this page, click on `New issue`. Fill out the fields on the right-hand side. Below is a brief description of what should be done. You can then click on `Submit new issue` to finish creating it. 

- `Assignees`: Team Member assigned to and responsible for the issue. Typically, there should only be one assignee. As often as possible, tasks should be focused enough to be completed by a single individual. It can be left blank if it is still unknown and the task is just currently being planned. 
- `Labels`: There are many pre-made labels. Select the one that is appropriate for your issue. Pre-made labels should be used if possible. New labels can be created by clicking `Edit labels`.
- `Projects`: Assign the issue to a project board if applicable. Projects will be discussed in `Section 6`.

Properly assigning this information will simplify tracking who is working on the issue, what the issue is, and what the issue is contributing towards. Once an issue is created, a new thread is made that other members can also comment on if help is needed.

### 5.3. Closing an Issue
An issue can be closed once the task has been completed. You will need to got back to the [issues page](https://github.com/riplaboratory/Kanaloa/issues). If there is a lot of issues currently open, click on the `Assignee` field and select your name so that you can filter it to issues assigned to you. Click on your issue and for most scenarios, you can click on `Close issue`. 

**Exceptions**:
- If you are not able to complete the issue, leave a comment and unassign yourself from it. 
- If the team determines the issue is not worth pursuing, leave a comment and close the issue.  

## 6. Project Board Usage
Project boards allows work to be organize and prioritized. It can be used to create a checklist or a roadmap to work towards completing a goal. It allows an easy view of task that still need to be done, task in progress, and task that have been done. This feature can be automated so that when a pull request goes through, your task is marked as done and the issue is closed. 

Project boards are a valuable tool to help with project management and workflow. It displays issues that still need to be done, in progress, and completed, along with the label and assignee of the issue. This visual allows the team to see where they are currently at with their completion of their goal and if the pace needs to be picked up to finish it on time. 

### 6.1. Project Board Naming Convention
Your project board name should be a short description of the overall goal you or your team is trying to achieve. Your description should be the general goal of the project. The issues you create for this project will be the goal broken down into more focused task that work towards the completion of it.

### 6.2. Creating a Project Board
To create a project board, first go to the [projects tab](https://github.com/riplaboratory/Kanaloa/projects). Click on `New project` then enter a name and description for it. For project template, choose `Automated kanban`. Doing this will automatically move issues and pull requests across To do, In progress and Done columns. For our purposes, these are the only three columns that will be needed.

- When a new issue is created, it will be automatically moved to the `To do` column.
- When an issue is closed, it will be automatically moved to the `Done` column.

If the issue is not for coding, but a task for the team, you can also create cards by clicking on the `+` in any of the columns. The text field supports markdown, so you can create the task similar to how you would for an issue's description.
-->
## Extra Resources
- [GitHub Desktop](https://desktop.github.com/) (this is a great UI for those starting to learn GitHub)
- [GitHub Documentation](https://docs.github.com/en)
- [GitHub CLI](https://cli.github.com/manual/) (for more advanced users and/or cases)
