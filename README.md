# perception

## <a name="contributing">Contributing</a>
### General Coding Philosophies / Making Changes

1. **Every change MUST be done on a new branch.**

  This project follows the simple [GitHub workflow](https://guides.github.com/introduction/flow/) to always keep the `master` branch deployable and free from buggy and untested code. The main principle of this is very straight forward: just create a new branch for every change you make to the code (these branches are called _feature branches_. If you are done with making changes in your newly created branch simply push the branch to the remote repository, create a new merge request in the GitLab and assign another person to review your changes (so that at least 4 eyes have seen and checked the code before it gets merged into the master branch).

  **Step 1**: _Make sure you are on the master branch and up-to-date_. In order to create a new branch, you should be on the master branch that is up-to-date with the remote repository. Just checkout the master branch and pull from the remote server (the remote server is called `origin` in this example).

  ```shell
  $ git checkout master
  $ git pull origin master
  ```

  **Step 2**: _Create a new branch_. Now simply create a new branch and change to it (fill `branch-name` with the actual name of the branch; see the [commit conventions](#commit-conventions) for the naming scheme of the branches).

  ```shell
  $ git checkout -b branch-name
  ```
2. **Every change MUST be reviewed.**

  In order to write better code with less bugs it is always necessary that at least one other person reviews the code. The idea is that, since you already created an own branch for your changes, you simply open a merge request in the GitHub and assign another person from the project to it to check the code (in the best case this person is somehow connected to the change). For more information on why code review is necessary, have a look [this article](https://www.sitepoint.com/the-importance-of-code-reviews/).

3. **Always update the documentation, if necessary.**

  To ensure that every part of the project is documented you MUST update the documentation, when necessary. This should land in the same merge request as the change itself, so it can also be reviewed all together. In this way, it also never creates technical debt to you/to the project (we all have enough of them...).
4. **Write meaningful code comments**

  _Good comments_ on newly written code is important. Not only for the people that didn't write the code and want to have a look at it afterwards (e.g. to review it or fix a bug), but also for you because most of the time you will not remember why you wrote the code the way you did 10 days later. To write good comments, you SHOULD read through the guidelines found [here](https://seesparkbox.com/foundry/lets_write_beautiful_css_comments) and try to apply them (even though the blog post talks about CSS comments, a lot of the guidelines are applicable to other languages as well). For example, you SHOULD always try to tell _why_ you are doing things the ways you do instead of _how/what_ (that should actually be visible from the code below the comments).
5. **Follow simple programming principles**

  Simple programming principles or code design patterns are a good way of keeping the code clean. You SHOULD read through the programming principles provided [here](https://webpro.github.io/programming-principles/) and try to apply them while adding new code. The most important ones are [KISS](https://webpro.github.io/programming-principles/#kiss), [YAGNI](https://webpro.github.io/programming-principles/#yagni), [DRY](https://webpro.github.io/programming-principles/#keep-things-dry) and the [Boy-Scout Rule](https://webpro.github.io/programming-principles/#boy-scout-rule), but also try to keep the others in mind as well. When you are applying the _Boy-Scout-Rule_ please do so in a seperate commit.

### Coding Styleguides and Naming Conventions
Coding styles and naming conventions are important to keep every part of the codebase readable and as always: consistency is key! So please do your very best to follow the rules below when adding new code to the project. For rules like soft-tabs and tab-size try to configure your editor to the guidelines. This will make it easier to follow them.

#### TODO

### <a name="commit-conventions">Commit conventions (Submitting Changes)</a>
1. **Create an issue first**

  If you find a bug in the code or something that is misbehaving, you SHOULD fill an issue in the bug tracker (https://trello.com/b/PquROHS6/suturo-2016) first. This ensures that everybody in the team is aware of the bug. Then use the labeling and assign functionalities to give it some more context and a category (see the [issue tracker section](#issue-tracker) for more information). If you want to resolve the issue, assign it to yourself. Only after creating the issue you should start fixing and creating a merge request. The issues and merge request MAY be in English.
2. **Use the branching model**

  For this project we use the most simple git branching model suitable for this codebase, the GitHub flow, which is explained [here](http://scottchacon.com/2011/08/31/github-flow.html) and [here](https://guides.github.com/introduction/flow/) in more detail. The main idea is as simple as creating a new branch from `master` for every change (while following the branch naming convention), creating a merge request for it, assigning a person to review it when it's done and let this person merge the branch into the `master`-branch. Please read both in-depth articles provided above before using it. A simple step-by-step solution on how to do it is explained in the [Contributing](#contributing) section.
3. **Use the branch naming convention**

  When creating a new branch to work on you MUST follow the branch naming convention. This ensures a homogeneous overview of all branches and allows for a quick scan of everything that is being worked on. The branches will always get a tag at the beginning. After that a short (very short, max. 5-6 words) description of the bug will be added. Both parts will be separated by a slash (`/`), while the single words in the short description will be separated by a hyphen (`-`). Therefore your branch name should follow this simple form: `tag/short-description`. You will be able to choose from the following tags:

  - `feature`: This tag MUST be used when adding a new feature to the codebase.
  - `fix`: This tag MUST be used when fixing a bug in an existing codebase.
  - `misc`: Everything that fits not one of the tags above MUST use this tag instead.
4. **Follow the commit message guidelines**

  In order to be able to easily scan through commit messages/history and the changes your MUST follow the commit message guidelines. The first line of the commit message MUST be concrete and written in imperative and present tense form ("Add feature ..." instead of "Added feature ..." or "Adds feature ..."). Also, the first line SHOULD not exceed 50 characters (this is usually a length after which the text is cut off in git management systems like GitLab or GitHub). After the first line a short description should follow in which you are more specific about the changes you made (include not only _what_ but more importantly _why_ you did it). You SHOULD try to line-break these descriptions at 72 characters (we are basically following the 50/72 format as [proposed by Tim Pope](http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html)). If the commit fixes a bug, also consider adding a reference to the GitLab issue. This will close the issue automatically if the commit gets merged into the master branch (see [here](http://docs.gitlab.com/ee/customization/issue_closing.html) for more information about the actual format of this reference). All commit messages MUST be written in English and the first letters MUST be in capitals.

  You SHOULD start your messages with an applicable emoji. This will ease the scanning of the messages even more and also categorize each commit. The emojis you are able to choose from is on of the following:

  - :unicorn: `:unicorn:` when adding a completely new feature
  - :muscle: `:muscle:` when improving the format/structure of the code
  - :art: `:art:` when changing/improving frontend/design
  - :rocket: `:rocket:` when improving performance
  - :memo: `:memo:` when writing docs
  - :bug: `:bug:` when fixing a bug
  - :fire: `:fire:` when removing code or files
  - :white_check_mark: `:white_check_mark:` when adding tests
  - :lock: `:lock:` when dealing with security
  - :ring: `:ring:` when working on dependencies
5. **Resolve all merge conflicts**

  Before assigning another developer for reviewing your merge request you MUST make sure that there are no merge conflicts with the `master` branch (GitHub does actually not allow merging when conflicts are present). If they are present it is your job to resolve them and not the reviewers.
6. **Delete merged branches**

  After a branch was successfully merged into the master branch all feature branches aren't very useful any longer. Therefore you MUST always delete those feature branches after the merge if not already done by the reviewer (in GitHub there is a button after merging, which will automatically delete the feature branch).

### How to review
To review the code others wrote is one of the main tasks of this project (you can read more information in the [Contributing](#contributing) section). The formal process is pretty easy and straight forward (the feature branch to be reviewed will be named `feature/to-be-reviewed` in this example).

**Step 1**: _Check out the feature branch_. In order to test the new/changed codebase you need to check out the feature branch of the merge request you are reviewing.

```shell
$ git checkout feature/to-be-reviewed
$ git pull origin feature/to-be-reviewed
```

**Step 2**: _Test/review the codebase_. Now that you have checked out the changed code you are able to test the new feature/the bugfix (the reviewing process also includes running the full test suite, which will be done by CI in the future). If something is wrong, reply to the discussion on the merge request and wait for a response from the creator. After the creator changed the code again, `pull` and test it again. This will be an ongoing cycle until everyone is happy with the added/deleted code.

**Step 3**: _Merge the feature branch_. At this point in time you should be able to merge the request in the GitHub. You SHOULD check the checkbox next to the merge button to delete the branch after merging. Then hit the merge button. Done.

