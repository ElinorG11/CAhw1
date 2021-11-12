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

    void updateHistory() {

    }

    void resetHistory(const uint32_t tag) {
        history_entry_array[tag] = 0;
    }
};

/*************************************************************************/
/*                     PREDICTION TABLE HANDLER                          */
/*************************************************************************/

class PredictionEntry {

};

class PredictionTable {

};

class PredictionMatrix {

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
    unsigned fsm_initial_state;

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

    void _update_entrance_info(const Indices &indices, const uint32_t target) {
        // Update tag and target in adequate entrance
        btb_entries_array[indices.btb_index].tag_identifier = indices.tag_index;
        btb_entries_array[indices.btb_index].target = target;
    }

    void _reset_predictor(const Indices &indices, uint32_t new_tag, uint32_t new_target) {
        // Update BTBEntry with new tag and target (?)
        /**************************************************************/
        /////////// TODO: Find out how to update the target if needed.
        /**************************************************************/
        uint32_t old_tag = btb_entries_array[indices.btb_index].tag_identifier;

        _update_entrance_info(indices, new_target);
        history_table.resetHistory(old_tag);

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
              btb_size(btbSize), history_size(historySize), tag_size(tagSize), fsm_initial_state(fsmState),
              is_history_global(isGlobalHist), is_fsm_global(isGlobalTable),
              share_mode(static_cast<SHARE_MODE>(Shared)), btb_entries_array(new BTBEntry[btbSize]),
              history_table(isGlobalHist, historySize) {

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
                return prediction_matrix.predict(...);

            } else {
                // The currently handled branch has the same btb index but DIFFERENT tag. Reset entry.
                _reset_predictor(...);
            }
        }

        // Entry does not exist.
        return false;
    }

    void updateBtb(uint32_t pc, uint32_t targetPc, bool taken, uint32_t pred_dst) {
        statistics->br_num++;
        if(predict(pc, &pred_dst) == taken) {
            return;
        } else {
            statistics->flush_num++;
        }
    }

    void getStat(SIM_stats &update_stat) {
        update_stat.br_num = statistics->br_num;
        update_stat.flush_num = statistics->flush_num;
        update_stat.size = statistics->size;
    }
};



/*************************************************************************/
/*                          TODO
 * 1. Implement calc_total_size to calculate total size of btb
 * 2. Implement PredictionMatrix class
 *    2.1 predict
 *    2.2 updateFsm
 *    2.3 resetFsm
 * 3. Implement PredctionTable class (per branch)
 *    3.1 predict
 *    3.2 updateFsm
 *    3.3 resetFsm
 * 4. Implement PredictionEntry class (per history)
 *    == Check if needed ==
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

