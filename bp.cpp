/* 046267 Computer Architecture - Spring 2020 - HW #1 */
/* This file should hold your implementation of the predictor simulator */

#include "bp_api.h"
#include <exception>
#include <bitset>
#include <iostream>
#include <cmath>

using namespace std;

/*************************************************************************/
/*              CONSTANTS, TYPDEFS AND OTHER VEGETABLES                  */
/*************************************************************************/

#define OPCODE_BITS_IDX 31

inline bool COMPARE(uint32_t a, uint32_t b) { return a == b; }

enum SHARE_MODE { NOT_SHARED = 0, LSB_SHARED = 1, MID_SHARED = 2};
enum STATE { SNT = 0, WNT = 1, WT = 2, ST = 3};

/*************************************************************************/
/*                       ADDRESS PARSER CLASS                            */
/*************************************************************************/
/*
 * Not sure if there's any need for it right now, perhaps it's better to
 * declare it inside the BTBEntry class
 */

/*************************************************************************/
/*                          HISTORY HANDLER                              */
/*************************************************************************/

class HistoryTable {

    bool is_history_global;
    uint8_t *history_entry_array;

public:
    HistoryTable(bool is_global_hist, const unsigned history_size) :
                is_history_global(is_global_hist), history_entry_array(new uint8_t[history_size]) {};

    ~HistoryTable() {
        delete[] history_entry_array;
    };

    uint8_t getHistory(const uint32_t tag) const {
        if(is_history_global) {
            return history_entry_array[0];
        }

        return history_entry_array[tag];
    }

    void updateHistory(uint32_t tag, bool taken) {
        history_entry_array[tag] << 1;
        if(taken) {
            history_entry_array[tag] += 1;
        }
    }

    void resetHistory(const uint32_t tag) {
        history_entry_array[tag] = 0;
    }
};

/*************************************************************************/
/*                     PREDICTION TABLE HANDLER                          */
/*************************************************************************/

class PredictionFsmPerHistory {
    STATE current_state;
public:
    PredictionFsmPerHistory() = default;
    ~PredictionFsmPerHistory() = default;

    bool predict() {
        if(current_state == SNT || current_state == WNT) {
            return false;
        } else {
            return true;
        }
    }

    void resetFsm(const STATE new_state) {
        current_state = new_state;
    }

    void updateFsm(bool taken) {
        int state_number = static_cast<int>(current_state);
        if(taken) {
            state_number++;
        } else {
            state_number--;
        }
        current_state = static_cast<STATE>(state_number);
    }
};

class PredictionTablePerBranch {
    PredictionFsmPerHistory *prediction_entries;
public:

    void _create_entries_array(unsigned history_size, STATE initial_state) {
        prediction_entries = new PredictionFsmPerHistory[history_size];

        // Cannot initialize arrays in cpp.
        for(int i = 0; i < history_size ; i++) {
            prediction_entries[i].resetFsm(initial_state);
        }
    }
    void _destroy_entries() {
        delete[] prediction_entries;
    }

    PredictionTablePerBranch() = default;
    ~PredictionTablePerBranch() = default;

    bool predict(uint8_t history_key) {
        return prediction_entries[history_key].predict();
    }

    void updateFsm(uint8_t history_key, bool taken) {
        prediction_entries[history_key].updateFsm(taken);
    }

    void resetFsm(uint8_t history_key, STATE initial_state) {
        prediction_entries[history_key].resetFsm(initial_state);
    }

};

class PredictionMatrix {
    bool is_table_global;
    PredictionTablePerBranch *prediction_table;

public:
    PredictionMatrix(uint8_t btb_size, unsigned history_size, STATE initial_state) {
        prediction_table = new PredictionTablePerBranch[btb_size];
        prediction_table->_create_entries_array(history_size, initial_state);
    };

    ~PredictionMatrix() {
        prediction_table->_destroy_entries();
        delete[] prediction_table;
    }


    bool predict(uint32_t tag_index, uint8_t history_key) {
        if(is_table_global) {
            return prediction_table[0].predict(history_key);
        }
        else {
            return prediction_table[tag_index].predict(history_key);
        }
    }

    void updateFsm(uint32_t tag_index, uint8_t history_key, bool taken) {
        if(is_table_global) {
            return prediction_table[0].updateFsm(history_key, taken);
        }
        else {
            return prediction_table[tag_index].updateFsm(history_key, taken);
        }
    }

    void resetFsm(uint32_t tag_index, uint8_t history_key, STATE initial_state) {
        if(is_table_global) {
            return prediction_table[0].resetFsm(history_key, initial_state);
        }
        else {
            return prediction_table[tag_index].resetFsm(history_key, initial_state);
        }
    }
};

/*************************************************************************/
/*                          PREDICTOR HANDLER                            */
/*************************************************************************/

/**************************    HELPER STRUCT TO STORE NECCESSARY INDICES    **************************/

struct Indices {
    uint8_t btb_index = -1; // Needs maximum of 5 bits for representation. Closest adequate uintX_t type is uint8_t.
    uint32_t tag_index = -1; // Needs maximum 30 bits for representation. Closest adequate uintX_t type is uint32_t.
    uint8_t shared_index = -1; // Needs maximum 3 bits for representation. Closest adequate uintX_t type is uint8_t.
};

struct BTBEntry {
    uint32_t tag_identifier = -1;
    uint32_t target = 0;
};

class BTBTable {

    /**************************************    BTB TABLE FIELDS    **************************************/

    unsigned btb_size;
    unsigned history_size;
    unsigned tag_size;
    STATE fsm_initial_state;

    bool is_history_global;
    bool is_fsm_global;
    SHARE_MODE share_mode;

    BTBEntry *btb_entries_array;
    HistoryTable history_table;
    PredictionMatrix prediction_matrix;
    SIM_stats *statistics;

    /*********************************    BTB TABLE HELPER FUNCTIONS    *********************************/

    unsigned _calc_total_size() {
        unsigned size = btb_size * (tag_size + 1 + (OPCODE_BITS_IDX + 1) + history_size + (2 * pow(2,history_size)));
        return size;
    }

    bool _btb_entry_exists(const uint8_t btb_index) const{
        return (btb_entries_array[btb_index].tag_identifier != -1);
    }

    void _insert_btb_entry(Indices &indices, uint32_t targetPc) {
        btb_entries_array[indices.btb_index].tag_identifier = indices.tag_index;
        btb_entries_array[indices.btb_index].tag_identifier = indices.tag_index;
        history_table.resetHistory(indices.tag_index);
        prediction_matrix.resetFsm(indices.tag_index, history_table.getHistory(indices.tag_index), fsm_initial_state);
    }

    void _update_entrance_info(const Indices &indices, const uint32_t target) {
        // Update tag and target in adequate entrance
        btb_entries_array[indices.btb_index].tag_identifier = indices.tag_index;
        btb_entries_array[indices.btb_index].target = target;
    }

    void _reset_btb_entry_upon_tag_collision(const Indices &indices, const uint32_t new_target) {
        // Update BTBEntry with new tag and target (?)
        /**************************************************************/
        /////////// TODO: Find out how to update the target if needed.
        /**************************************************************/
        uint32_t old_tag = btb_entries_array[indices.btb_index].tag_identifier;
        uint8_t old_history_key = is_fsm_global ? old_tag ^ history_table.getHistory(old_tag) : history_table.getHistory(old_tag);

        _update_entrance_info(indices, new_target);
        history_table.resetHistory(old_tag);
        prediction_matrix.resetFsm(indices.tag_index, history_table.getHistory(indices.tag_index), fsm_initial_state);

    }

    uint32_t _get_bits_by_indices(const uint32_t address, const int start, const int end) {
        uint32_t decoded_bits = address;
        decoded_bits <<= (OPCODE_BITS_IDX - end);
        decoded_bits >>=  (start + OPCODE_BITS_IDX - end);
        return decoded_bits;
    }

    void _decode_indices(const uint32_t branch_pc, Indices &indices) {

        // Extract btb entry index. Opcodes are aligned to 4 bytes.
        // Ignore the two LS bits since they are always equal to zero.
        indices.btb_index = _get_bits_by_indices(branch_pc, 2, 2 + btb_size);
        indices.tag_index = _get_bits_by_indices(branch_pc, 2 + btb_size, 2 + btb_size + tag_size);

        uint8_t pc_bits_to_share;
        uint8_t history_bits = history_table.getHistory(indices.tag_index);
        switch (share_mode) {
            case NOT_SHARED:
                indices.shared_index = -1;
                break;
            case LSB_SHARED:
                pc_bits_to_share = _get_bits_by_indices(branch_pc, 2, 2 + history_size);
                indices.shared_index = pc_bits_to_share ^ history_bits;
                break;
            case MID_SHARED:
                pc_bits_to_share = _get_bits_by_indices(branch_pc, 16, 16 + history_size);
                indices.shared_index = pc_bits_to_share ^ history_bits;
                break;
            default:
                indices.shared_index = -1;
        }
    }

public:
    BTBTable( unsigned btbSize,
              unsigned historySize,
              unsigned tagSize,
              unsigned fsmState,
              bool isGlobalHist,
              bool isGlobalTable,
              int Shared ) :
              btb_size(btbSize), history_size(historySize), tag_size(tagSize), fsm_initial_state(static_cast<STATE>(fsmState)),
              is_history_global(isGlobalHist), is_fsm_global(isGlobalTable),
              share_mode(static_cast<SHARE_MODE>(Shared)), btb_entries_array(new BTBEntry[btbSize]),
              history_table(isGlobalHist, historySize),
              prediction_matrix(btbSize, historySize, static_cast<STATE>(fsmState)) {

        statistics->br_num = 0;
        statistics->flush_num = 0;
        statistics->size = _calc_total_size();
    };

    ~BTBTable() {
        // No need to destroy entries first since we store the entry itself and not a pointer
        // to the entry, thus, the D'tor will be called automatically
        delete[] btb_entries_array;
    }


    bool predict(const uint32_t branch_pc, uint32_t *target) {

        // Extract all necessary indices.
        auto *indices = new Indices();
        _decode_indices(branch_pc, *indices);

        // Check if entrance already exist
        if(COMPARE(btb_entries_array[indices->btb_index].tag_identifier,-1)) {

            // Entry exists. Now need to make sure it has the same tag, else need to update.
            if(COMPARE(btb_entries_array[indices->btb_index].tag_identifier, indices->tag_index)) {

                // The currently handled branch has the same btb index and same tag.
                // No need to reset, just get index to the fsm and return the prediction and target.
                *target = btb_entries_array[indices->btb_index].target;
                uint8_t history_key = is_fsm_global ? indices->shared_index : history_table.getHistory(indices->tag_index);
                return prediction_matrix.predict(indices->tag_index,history_key);

            } else {
                // The currently handled branch has the same btb index but DIFFERENT tag. Reset entry.
                _reset_btb_entry_upon_tag_collision(*indices,...);
            }
        }

        // Entry does not exist.
        return false;
    }

    void updateBtb(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
        statistics->br_num++;
        auto *indices = new Indices();
        _decode_indices(pc, *indices);

        // Check if entry exists
        if(_btb_entry_exists(indices->btb_index)){

            // Entry exists. Update statistics
            if((!taken && (pred_dst != pc+4)) || (taken && (pred_dst == pc+4)) || (taken && (pred_dst != targetPc))) {
                statistics->flush_num++;
            }

            // Update BTB
            _update_entrance_info(*indices,targetPc);
            uint8_t history_key = is_fsm_global ? indices->shared_index : history_table.getHistory(indices->tag_index);
            history_table.updateHistory(indices->tag_index, taken);
            prediction_matrix.updateFsm(indices->tag_index, history_key, taken);
        } else {
            _insert_btb_entry(*indices, targetPc);
        }

        delete indices;
    }

    void getStat(SIM_stats &update_stat) {
        update_stat.br_num = statistics->br_num;
        update_stat.flush_num = statistics->flush_num;
        update_stat.size = statistics->size;
    }
};



/*************************************************************************/
/*                          TODO
 * Check how to resolve target on tag collision in BTBTable.predict()
 */
/*************************************************************************/




BTBTable* branch_predictor;

int BP_init(unsigned btbSize, unsigned historySize, unsigned tagSize, unsigned fsmState,
			bool isGlobalHist, bool isGlobalTable, int Shared){
    if ((btbSize != 1 && btbSize != 2 && btbSize != 4 && btbSize != 8 && btbSize != 16 && btbSize != 32) ||
    (historySize < 1 || historySize > 8) || (tagSize < 1 || tagSize > 30 - log2(btbSize)) ||
    (fsmState < 0  || fsmState > 3))
        return -1;
    try{
        branch_predictor = new BTBTable(btbSize, historySize, tagSize, fsmState, isGlobalHist,
                           isGlobalTable, Shared);
    } catch (bad_alloc&) {
        return -1;
    }
    return 0;
}

bool BP_predict(uint32_t pc, uint32_t *dst){
    if (branch_predictor->predict(pc, dst)){
        return true;
    } else {
        *dst = pc + 4;
        return false;
    }
}

void BP_update(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst){
    branch_predictor->updateBtb(pc, targetPc, taken, pred_dst);
	return;
}

void BP_GetStats(SIM_stats *curStats){

    branch_predictor->getStat(*curStats);

    // By TA's documentation this function will be called once at the end of a test set
    // and may be used to free allocated data
    delete branch_predictor;
	return;
}

