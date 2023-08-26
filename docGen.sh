# remove previous source files
rm ./docs/source/scripts.rst
rm ./docs/source/robot_command.rst
rm ./docs/source/robot_goal_result_count_srv.rst
rm ./docs/source/robot_info_print.rst
rm -r ./docs/html

# sphinx-apidoc -e -o ./docs/source ./rt1_ass2/scripts/
sphinx-apidoc -e -t ./docs/source/_templates/apidoc -o ./docs/source ./rt1_ass2/scripts/

cd ./docs/source
# remove unused files
rm modules.rst scripts.rst

# remove prefix scripts for source files
for sourceFile in *robot_*.rst; do
    newName=$sourceFile
    mv $sourceFile ${newName#"scripts."}
done

cd ..
make clean
make html
mkdir html
cp -r ./build/html .
cd ..
