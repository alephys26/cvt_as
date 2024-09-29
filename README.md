# cvt_as
## IIT Jodhpur | CSL7560: Autnomous Systems
## Course Assignment 1: A Multiagent System for Campus Virtual Tour  

The **main** branch is secured and no commits can be pushed to it without raising a Pull Request which is also approved by anyone of the collaborators. So create a new branch for any new task and raise a PR for it and wait for approval.
## Branch Name Instructions  
Format: **Tx__Task**  
where  
- _x_ is 1 for Shrivastava, 2 for Anshu, 3 for Manan, 4 for Mangal.  
- _Task_ is the task title. If it has multiple words kindly write them separated by underscores and in capital case.
  

For example, Yash creates a branch for readme filling through which this readme is created, then the branch is named ***B1__Initial_Readme_Updates***.  
As a good practice, remember to keep two underscores after _Bx_ and single underscore for each word in task as shown in above example.  
So, the command for creating the branch will be of the form  
`git branch "Bx__Task"`  
To switch to this branch, use  
`git checkout "Bx__Task"`  
To do the above two actions together in a single command   
`git checkout -b "Bx__Task"`  

## Commit Instructions  
The commit message format should be  The commits should have a message of the form ***Bx__Task: Description*** or ***BranchName: Description*** where _Description_ is in  
1. Present Tense and Imperative Mood
2. Tells what eactly has been done in the commit  
So that the task done is clear and the branch is clear. Remember to keep the message tense in imperative mood.  
For example, **git commit -m "B1__Initial_ReadMe_Updates: ReadMe has Instructions"**  

## Pull Request Instructions  
The **main** branch is protected and requires at least one approving review on a pull request to push changes to it.  
Try to keep each PR to at *max 150 lines* of code.  
To push your branch, use  
`git push origin HEAD` or `git push origin "Bx__Task"`  
Make multiple PRs for the same task if it exceeds the above limit. Do not change the branch name to a new branch name for the same task. The branch will get deleted on remote after merging but on local it can be kept safe or a fresh branch with the old name can be made.  

## Other Instructions
Use **autopep8** formatter for code formatting with tab = 4 spaces and UTF-8 encoding.  
Keep variables as private in classes.  
Use setter and getter methods to handle class variables.  
Prefer loops over lambda functions.  
Use array comprehension for simple tasks.  

> _Yash Shrivastava, B21CS079_  
  _Anshu Raj, B21AI048_  
  _Manan Jain, B21AI021_  
  _Yash Mangal, B21AI047_  
