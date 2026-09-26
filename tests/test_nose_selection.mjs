import assert from 'node:assert/strict';
import test from 'node:test';
import {noseSelection, removeNoseMotor, simulatorNoseSelected} from '../ui/nose_selection.js';
test('deleting before or within nose selection preserves identity',()=>{
 assert.deepEqual(removeNoseMotor([8,9],6),[7,8]);
 assert.deepEqual(removeNoseMotor([7,8],6),[6,7]);
 assert.deepEqual(removeNoseMotor([6,7],6),[6]);
});
test('explicit empty selection never silently selects other motors',()=>{
 assert.deepEqual(noseSelection([{pos:[1,0,0]}],[]),[]);
});
test('simulator mode cannot override hardware routing',()=>{
 assert.equal(simulatorNoseSelected('sitl',{enabled:true}),true);
 assert.equal(simulatorNoseSelected('hitl',{enabled:true}),false);
 assert.equal(simulatorNoseSelected(null,{enabled:true}),false);
});
